import numpy as np
import control
import scipy.linalg as la
import matplotlib.pyplot as plt
from pytransform3d.trajectories import *

# from enoid_walk.scripts.inverse_kinematic import InverseKinematic
from preview_controller import *
from inverse_kinematic import *
from forward_kinematic import *

debug = 1

class PreviewControl:
    def __init__(self, com_height, com_swing, x_offset, foot_distance):

        self.dt = 0.01
        self.previewStepNum = 120
        self.zc = com_height
        self.g = 9.8
        self.x_offset = x_offset
        self.com_swing = com_swing
        self.foot_y = foot_distance + (0.0385)
        self.first = False

        # Kajita's Book P:144 Eq:4.72
        self.A = np.matrix([[1,self.dt, (self.dt ** 2) / 2],
                            [0, 1, self.dt],
                            [0, 0, 1]])
        self.B = np.matrix([(self.dt ** 3) / 6, (self.dt ** 2) / 2, self.dt]).T
        self.C = np.matrix([1, 0, -self.zc / self.g])

        # Kajita's Book P:145 Eq:4.77
        self.A_tilde = np.hstack((np.matrix([1, 0, 0, 0]).T, np.vstack((self.C * self.A, self.A))))
        self.B_tilde = np.vstack((self.C * self.B, self.B))
        self.C_tilde = np.matrix([1., 0., 0., 0.])

        self.Q = np.matrix([[1,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0]])

        self.R = np.matrix([1e-7])

        # Kajita's Book P:144 Eq:4.75
        P, _, K = control.dare(self.A_tilde, self.B_tilde, self.Q,self.R)

        # Kajita's Book P:146 Eq:4.80
        self.Ks = K[0,0]
        self.Kx = K[0,1:4]

        ######==========GAIN=============######
        self.G = np.zeros(self.previewStepNum)
        self.G[0] =-self.Ks

        Ac_tilde = self.A_tilde - self.B_tilde * K
        I_tilde = np.matrix([1., 0., 0., 0.]).T
        X_tilde = -Ac_tilde.T * P * I_tilde

        for i in range(1, self.previewStepNum):
            self.G[i] = la.inv(self.R + self.B_tilde.T * P * self.B_tilde) * self.B_tilde.T * X_tilde
            X_tilde = Ac_tilde.T * X_tilde
        ######===========================######

        self.cnt = 1
        #Jarak dari Hip ke CoM
        self.hip_offset = 0.0385 + self.com_swing

        #Prediksi State
        self.x = np.matrix(np.zeros(3)).T
        self.y = np.matrix(np.zeros(3)).T

        #State untuk plot v_CoM
        self.one_step = False
        self.x0 = np.matrix(np.zeros(3)).T
        self.y0 = np.matrix(np.zeros(3)).T

        #ZMP State
        self.px_ref = []
        self.py_ref = []

        #Foot Step (Kenapa Butuh 3)
        #Foot step ini yang selalu di append dan pop di fungsi update_footstep()
        self.footstep = [[0.0, -self.hip_offset],
                         [0.0, self.hip_offset],
                         [0.0, -self.hip_offset]]

        self.support_foot = -1

        self.com_pose = np.matrix([0.0, 0.0, 0.0], dtype=float)
        self.v_com_pose = np.matrix([0.0, 0.0, 0.0], dtype=float)
        self.l_foot_pose = np.matrix([0.0, self.hip_offset, 0.0], dtype=float)
        self.r_foot_pose = np.matrix([0.0, -self.hip_offset, 0.0], dtype=float)
        self.cur_l_foot_pose = np.matrix([0.0, self.hip_offset, 0.0], dtype=float)
        self.cur_r_foot_pose = np.matrix([0.0, -self.hip_offset, 0.0], dtype=float)
        self.r = []
        self.l = []
        # Bezier Control Point
        self.p_start = np.matrix([0.0, 0.0, 0.0], dtype=float)
        self.p_cnt = np.matrix([0.0, 0.0, 0.0], dtype=float)
        self.p_end = np.matrix([0.0, 0.0, 0.0], dtype=float)

        #Parameter
        self.cmd_x = 0.025
        self.cmd_y = 0.0
        self.cmd_a = np.radians(0)

        self.sx = 0.0
        self.sy = 0.0
        self.sa = 0.0

        self.swing_height = 0.035

        # Walking Timing Pzrameter
        self.t_step = 0.3
        self.dsp_ratio = 0.1
        self.t_dsp = self.dsp_ratio * self.t_step
        self.t_ssp = (1.0 - self.dsp_ratio) * self.t_step
        self.t = 0.0

        self.t_bez = 0.0
        self.dt_bez = 1 / (self.t_ssp / self.dt)

        # 0 : DSP, 1 : SSP
        self.walking_phase = 0

        self.walking_ready = False

        # custom for fk
        self.com_trajectory = []

    def update_foot_trajectory(self):
        """
        Fungsi yang dipanggil untuk memperbarui trajektori kaki bezier curve.
        .........

        Attributes
        ---------
        self.t : float
            Parameter waktu yang diinisiasi dengan nilai 0.0 dan akan melakukan increment sebesar self.dt (self.dt = 0.01)
        self.support_foot : int
            Nilai yang digunakan untuk merepresentasikan kaki kiri (1) dan kaki kanan (-1)
        
        *********
        ADDITIONAL
        self.p_start, self.p_end, self.p_cnt digunakan dalam fungsi update_footstep(), menghasilkan path yang disimpan dalam
        self.l_foot_pose dan self.r_foot_pose berupa matriks 1x3 (x y z)
        *********

        self.p_start : matrix
            -> Beranggotakan matriks 1x3
            Digunakan untuk menyimpan nilai self.cur_l_foot_pose (pose kaki saat ini) 
        self.p_end : matrix
            -> Beranggotakan matriks 1x3
            Digunakan untuk menyimpan nilai (self.footstep[1][0], self.footstep[1][1], 0)
            -> self.footstep diinisiasi sebagai matriks 3x0, posisi x dan y
            -> matriks dari self.footstep akan selalu di append dengan value [dx, dy] di fungsi update_footstep()
            -> value dari self.footstep[0] akan di pop setelah update_footstep() dipanggil sebanyak 30x 
        self.p_cnt : matrix
            -> Beranggotakan matriks 1x3
            Digunakan untuk menyimpan nilai (self.p_start[0, 0] + (self.p_end[0, 0] - self.p_start[0, 0]) / 2, -self.hip_offset,
                                        2 * self.swing_height)
            -> Setelah diamati, hasil kalkulasi nilai x pada self.cnt selalu += 0.075
        self.walking_phase : int
            Digunakan untuk menyatakan kondisi DSP (0) dan SSP (1)
        self.t_bez : float
            Parameter waktu bezier yang diinisiasi dengan nilai 0.0 dan akan melakukan increment 
            sebesar self.dt_bez (self.dt_bez = 0.037)
        self.walking_ready : Boolean
            Bernilai True apabila nilai G_x dan G_y sudah di += sampai iterasi self.previewStepNum pada self.preview_control 
        self.first : Boolean
            Bernilai True apabila kondisi kaki sedang DSP dan self.support_foot = 1 (kiri)
        self.cur_l_foot_pose : matrix
            -> Beranggotakan matriks 1x3
            Digunakan untuk menyimpan nilai list array self.footstep saat ini, karena self.footstep akan diupdate pada
            fungsi self.update_footstep()
        self.l : list array
            -> Beranggotakan list array 1x3
            Digunakan untuk menyimpan dan appen nilai ([self.l_foot_pose[0,0], self.foot_y, self.l_foot_pose[0,2]]).
            Nilai pada self.l[0] akan di pop apabila panjang list = self.previewStepNum
        self.previewStepNum : int
            Bernilai 120, digunakan sebagai parameter menyimpan nilai posisi bezier curve pada self.l dari self.l_foot_pose
        
        
        ---------

        QUESTION
        1.  Kenapa reset DSP tiap < 0.03 atau > 0.27 (SSP selama 0.27 ini darimana??)
            -> tunning
        2.  Kenapa list self.footstep di pop setelah 30x append matriks??
            -> list akan di pop setiap kelipatan 30 karena langkah selesai pada t.step 0.3. 
            Asumsinya adalah self.cnt akan +=1 setiap sekali looping, yang mana self.dt apabila dilooping 30x 
            bernilai t.step = 0.3
        3.  Kenapa self.t_bez ditambah tiap self.dt_bez (1 / 27), hubungan dengan SSP?
            -> biar nilai incrementnya sebanyak self.t_ssp (0.27).
            biar pas akhirnya nilai t_bez += self.dt_bez =/mendekati 1 apabila dilooping sebanyak 27x,
            KARENA SATU KALI LOOPING ASUMSI TIME ADALAH 0.01
        4.  Kenapa ada self.update_foot_path() dan sel.update_footstep()?
            -> self.update_foot_path() menghasilkan append self.footstep(dx, dy) dari rumus kajita.
                nilai self.footstep() digunakan untuk p_start, p_end, p_cnt (YANG MASIH BERUPA TITIK 3 STATIS) yang
                digunakan pada bezier curve sebagai posisi awal, akhir, tengah.
            -> self.update_footstep() menghasilkan POSISI X Y Z  parabola (digunakan untuk melangkah, titik dinamis
                karna dieksekusi sebanyak 27 titik sampai t=1)

        """

        # State Untuk Update Start dan End Foot Trajectory
        if self.t == 0:
            if self.support_foot == 1: #Kiri
                self.p_start = self.cur_l_foot_pose
                self.p_end = np.matrix([self.footstep[1][0], self.footstep[1][1], 0])
                self.p_cnt = np.matrix([self.p_start[0, 0] + (self.p_end[0, 0] - self.p_start[0, 0]) / 2, -self.hip_offset,
                                        2 * self.swing_height])
                # print("x: {} y: {}".format(self.p_start[0, 0], self.p_start[0, 0] + self.p_end[0, 0] - self.p_start[0, 0] / 2))
            else:
                self.p_start = self.cur_r_foot_pose
                self.p_end = np.matrix([self.footstep[1][0], self.footstep[1][1], 0])
                self.p_cnt = np.matrix([self.p_start[0, 0] + (self.p_end[0, 0] - self.p_start[0, 0]) / 2, self.hip_offset,
                                        2 * self.swing_height])

                            
        # State Untuk Reset Fase Walking Bezier
        # self.t += 0.01, if self.t < 0.03 or self.t >= 0.3
        if self.t < (self.t_dsp / 2.0) or self.t >= (self.t_dsp / 2.0 + self.t_ssp):
            self.walking_phase = 0
            self.t_bez = 0
            if self.walking_ready:
                if self.support_foot == 1:
                    self.first = True
        else:
            self.walking_phase = 1
            if self.support_foot == 1: # KIRI
                self.cur_l_foot_pose[0, 0] = self.footstep[0][0]
                self.cur_l_foot_pose[0, 1] = -self.footstep[0][1]
                self.cur_l_foot_pose[0, 2] = 0

                """Kenapa banyak variable karena butuh digunakan untuk perkalian matriks, sedangkan 
                bentuknya self.footstep masih berbentuk list array karena akan di pop"""

                # self.update_foot_path() menghasilkan matriks 1x3, berisi posisi x y z 
                self.r_foot_pose = self.update_foot_path()
            else:

                self.cur_r_foot_pose[0, 0] = self.footstep[0][0]
                self.cur_r_foot_pose[0, 1] = -self.footstep[0][1]
                self.cur_r_foot_pose[0, 2] = 0

                # self.update_foot_path() menghasilkan matriks 1x3, berisi posisi x y z
                self.l_foot_pose = self.update_foot_path()
        # perbedaan pada self.l_foot_pose hanya pada nilai y ([0, 1]) nya, dimana y bernilai konstan : 0.154, 
        # sedangkan self.foot_y bernilai konstan : 
        self.l.append([self.l_foot_pose[0,0], self.foot_y, self.l_foot_pose[0,2]])
        self.r.append([self.r_foot_pose[0,0], -self.foot_y, self.r_foot_pose[0,2]])

        self.t_bez += self.dt_bez
        if(len(self.l) == self.previewStepNum):
            self.l.pop(0)
            self.r.pop(0)

        self.r_foot = np.matrix([self.r[0]])
        self.l_foot = np.matrix([self.l[0]])

    def update_foot_path(self):
        """
        Fungsi yang dipanggil untuk membuat trajektori bezier curve.
        Untuk melihat rumus asli bezier bisa dikonversi menjadi representasi matriks buka troubleshooting_documentation.py
        masalah no 3.
        .........

        Attributes
        ---------
        self.t_bez : float
            Parameter waktu yang akan inc setiap self.dt_bez (0.037) sebanyak 27x dalam satu langkah
        t : matrix
            -> Beranggotakan matriks 1x3
            Digunakan sebagai penjabaran rumus bezier curve dalam representasi matriks
        coef : matrix
            -> Beranggotakan  matriks 3x3
            Digunakan sebagai penjabaran rumus bezier curve dalam representasi matriks
        point : matrix
            -> Beranggotakan matriks 3x0
            Digunakan sebagai matriks P0 P1 P2 dalam rumus bezier curve
        path : matrix
            -> Beranggotakan matriks 1x3
            Merupakan hasil dari fungsi yang memberikan posisi x y z bezier curve
        
        """
        # self.t_bez tidak boleh melebihi 1 karena rumus bezier 3 dimensi maksimal t = 1
        # kalo bingung lihat di troubleshooting_documentation, number 3
        if(self.t_bez >= 1):
            self.t_bez = 1

        t = np.matrix([1,  self.t_bez,  self.t_bez**2])
        # bezier curve coef matrix
        coef = np.matrix([[ 1, 0, 0],
                          [-2, 2, 0],
                          [ 1,-2, 1]])
        point = np.vstack((self.p_start, self.p_cnt, self.p_end))

        path = t * coef * point
     
        return path

    def swap_support_foot(self):
        # 1 => Kiri # -1 => Kanan
        if self.support_foot == 1:
            self.support_foot = -1
        else:
            self.support_foot = 1

        self.one_step = True

    def update_footstep(self):
        """
        Fungsi ini digunakan untuk mengupdate nilai self.footstep (zmp)
        .........

        Methods
        ---------
        self.swap_support_foot()
            Menjadi switcher kaki kiri dan kanan


        Attributes
        ---------
        self.cnt : int
            Digunakan sebagai parameter kondisi True untuk melanjutkan algoritma
            -> Kenapa apabila self.cnt = 30 maka kondisi True, karena tiap satu kali looping = self.dt (0.01).
                Dan satu kali langkah self.t_step = 0.3, maka self.dt * 30 = 0.3 (satu langkah)
        self.footstep : list array
            -> Berupa array 1x0
            Digunakan untuk menyimpan nilai zmp x dan y
        self.support : int
            Digunakan untuk menyatakan kaki support kiri (1) dan kanan (-1)
        self.cmd_x : float
            Digunakan sebagai parameter kecepatan jalan robot sumbu x
        self.sx : float
            Digunakan untuk menyimpan nilai self.cmd_x
        self.sy : float
            Digunakan untuk menyimpan nilai  -2 * self.hip_offset + self.cmd_y
        self.sa : float
            Digunakan untuk menyimpan += self.cmd_a
        dx : float
            Menyimpan hasil path zmp posisi x
        dy : float
            Menyimpan hasil path zmp posisi y
        """
        # if parameter berisi self.cnt (inisiasi = 1) % 30 == 0:
        # self.cnt akan += 1, nilai if dieksekusi apabila self.cnt kelipatan 30.
        if self.cnt % int(self.t_step / self.dt) == 0:
            self.footstep.pop(0)

            if self.support_foot == 1:
                self.sx = self.cmd_x
                self.sy = -2 * self.hip_offset + self.cmd_y
                self.sa += self.cmd_a
                # Kajita's Book P:132 Eq:4.61
                # dx merupakan posisi x, asumsi karena hasil valuenya meningkat terus 
                # dy merupakan posisi y, hasilnya sama -0.154
                dx = self.footstep[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
                dy = self.footstep[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
                self.footstep.append([dx, dy])
                # print(self.footstep[-1][0])
            else:
                self.sx = self.cmd_x
                self.sy = 2 * self.hip_offset + self.cmd_y
                self.sa += self.cmd_a
                # Kajita's Book P:132 Eq:4.61
                dx = self.footstep[-1][0] + np.cos(self.sa) * self.sx + (-np.sin(self.sa) * self.sy)
                dy = self.footstep[-1][1] + np.sin(self.sa) * self.sx + np.cos(self.sa) * self.sy
                self.footstep.append([dx, dy])

            self.swap_support_foot()

        if self.cnt % int(self.t_step / self.dt) == 1 or self.cnt / int(self.t_step / self.dt) == 0:
            self.x0 = self.x
            self.y0 = self.y
            # print(self.cnt)

        self.cnt += 1

    def update_pose(self):
        """
        Fungsi ini digunakan untuk memanggil self.update_foot_trajectory() dan menambahkan atau mengurangi isi dari
        list self.px_ref dan self.py_ref
        .........

        Methods
        ---------
        self.update_foot_trajectory()
            Menghasilkan trajektori bezier curve


        Attributes
        ---------
        self.px_ref : list array
            Digunakan untuk menyimpan dan append nilai self.footstep([0][0]) (zmp x)
        self.py_ref : list array
            Digunakan untuk menyimpan dan append nilai self.footstep([0][1]) (zmp y)
       
        """
        # Perbaharui Trajektori Kaki
        self.update_foot_trajectory()

        # Hapus Refrensi Paling Depan Kalau Refrensi Sudah Sebanyak Yang Diinginkan
        if len(self.px_ref) == self.previewStepNum:
            self.px_ref.pop(0)
            self.py_ref.pop(0)

        # Tambah Refrensi Paling Belakang dari Footstep
        self.px_ref.append(self.footstep[0][0])
        self.py_ref.append(self.footstep[0][1])

    def update_preview_control(self):
        """
        Fungsi ini digunakan untuk memperbaharui trakjektori CoM
        .........

        Attributes
        ---------
        self.px_ref : list array
            Berisi self.footstep[0][0] (zmp x)
        self.py_ref : list array
            Berisi self.footstep[0][1] (zmp y)
        self.A : matrix
            -> Berisi matriks 3x3
        Self.B : matriks
            -> Berisi matriks 1x3
            Nilai yang disimpan : (1.666666666666667e-07, 5e-05, 1e-02)
        self.C : matrix
            -> Berupa matriks 1x3 
            Nilai yang disimpan : (1, 0, -0.0234)   
        self.x : matriks
            -> Berupa matriks 3x0
        xe : float
            Nilai yang disimpan : (self.px_ref[0] - self.C * self.x)
        """
        
        #Perbaharui Trajektori CoM
        xe = self.px_ref[0] - self.C * self.x
        ye = self.py_ref[0] - self.C * self.y
        # print("xe: {}, ye: {}".format(xe, ye))

        G_x, G_y = 0,0

        for j in range(0, self.previewStepNum):
            G_x += self.G[j] * self.px_ref[j]
            G_y += self.G[j] * self.py_ref[j]
            self.walking_ready = True
            # print(self.G[j])

        # Kajita's Book P:146 Eq:4.80
        ux = -self.Ks * xe - self.Kx * self.x - G_x
        uy = -self.Ks * ye - self.Kx * self.y - G_y

        # Kajita's Book P:145 Eq:4.77
        self.x = self.A * self.x + self.B * ux
        self.y = self.A * self.y + self.B * uy
        # print("x: {}, y: {}".format(self.x, self.y))

        # Digunakan untuk menghitung v = x-x0 / t_step
        if self.one_step:
            self.v_com_pose[0,0], self.v_com_pose[0,1], self.v_com_pose[0,2] = ((self.x[0,0] - self.x0[0,0]) / self.t_step, 
            (abs(self.y[0,0] - self.y0[0,0]) / self.t_step), (self.zc - self.zc) / self.t_step)
            self.one_step = False

        self.com_pose[0,0], self.com_pose[0,1], self.com_pose[0,2] = self.x[0,0] + self.x_offset, self.y[0,0], self.zc
        
        if(not self.first):
            self.com_pose -= 0.0065
    def update_walking_pattern(self):
        """
        Fungsi ini digunakan untuk
        .........

        Methods
        ---------
        self.update_pose()
            Menghasilkan trajektori bezier curve dan menambah atau mengurangi anggota list dari self.px_ref (zmp x)
            dan self.py_ref (zmp y)
        self.update_preview_control()

        self.update_footstep()
            Menghasilkan prediksi nilai zmp x dan y yang akan di append pada self.footstep dan melakukan switching
            kaki support kanan dan kiri.


        Attributes
        ---------
        self.t : float
            Parameter waktu yang akan di inc setiap self.dt (0.01)
        """

        self.update_pose()

        if len(self.px_ref) == self.previewStepNum:
            self.update_preview_control()

        self.t += self.dt
        # jika sudah satu kali melangkah, reset
        if self.t > self.t_step:
            self.t = 0
        # Perbaharui ZMP
        self.update_footstep()

    def run(self):
        """
        Fungsi yang dipanggil untuk menjalankan preview_controller.
        Output yang dihasilkan adalah plot hasil CoM dan ZMP pada x dan y.
        .........

        Attributes
        ---------
        t_sim : float
            Digunakan sebagai parameter waktu lamanya run() akan dijalankan
        t : float
            Digunakan sebagai waktu yang melakukan increment sampai mencapai t_sim
        com_x : list 
            Digunakan untuk menyimpan nilai self.x[0, 0] (self.x = prediksi state x)
        com_y : list
            Digunakan untuk menyimpan nilai self.y[0, 0] (self.y = prediksi state y)
        px : list
            Digunakan untuk menyimpan nilai self.px_ref[0] (self.px_ref = refrensi zmp x, zmp state)
        py : list
            Digunakan untuk menyimpan nilai self.py_ref[0] (self.py_ref = refrensi zmp y, zmp state)
        com_trajectory : list 
            -> Beranggotakan matriks 1x7, +=1 setiap looping
            Digunakan untuk menyimpan nilai ([self.x[0, 0], self.y[0,0], 0.19, 0, 0, 0, 0)
        r_trajectory : list
            -> Beranggotakan matriks 1x7, +=1 setiap looping
            Digunakan untuk menyimpan nilai ([self.r_foot[0, 0], self.r_foot[0, 1], self.r_foot[0, 2], 0, 0, 0, 0)
            -> self.r_foot merupakan matriks 1x3 hasil dari self.r
            -> self.r merupakan matriks 1x3 hasil dari self.r_foot_pose (yang berbeda adalah nilai y = self.foot_y)
            -> self.r_foot_pose merupakan hasil dari update_walking_pose() menghasilkan path : matriks 1x3, posisi x y z
        l_trajectory : list
            -> Beranggotakan matriks 1x7, +=1 setiap looping
            -> 7 matriks karena untuk diplot butuh direpresentasika ke quarternion (x, y, z, x0, y2, z2, w)
            Digunakan untuk menyimpain nilai [self.l_foot[0, 0], self.l_foot[0, 1], self.r_foot[0, 2], 0, 0, 0, 0]
            *self.l_foot merupakan matriks 1x3 hasil dari self.l
            *self.l merupakan matriks 1x3 hasil dari self.l_foot_pose (yang berbeda adalah nilai y = self.foot_y)
            *self.l_foot_pose merupakan hasil dari update_walking_pose() menghasilkan path : matriks 1x3, posisi x y z
        
        """

        t_sim = 3
        t = 0
        com_x = []
        com_y = []

        px = []
        py = []

        com_trajectory = []
        r_trajectory = []
        l_trajectory = []

        while t < t_sim:
            self.update_walking_pattern()
            t += self.dt
            com_x.append(self.x[0, 0])
            com_y.append(self.y[0, 0])

            px.append(self.px_ref[0])
            py.append(self.py_ref[0])

            com_trajectory.append([self.x[0, 0],self.y[0, 0],0.19,0,0,0,0])

            print(com_trajectory)

            r_trajectory.append([self.r_foot[0,0], self.r_foot[0,1], self.r_foot[0,2],0,0,0,0])
            l_trajectory.append([self.l_foot[0,0], self.l_foot[0,1], self.l_foot[0,2],0,0,0,0])
            # if self.one_step:
            #     print("V_x_CoM: {}, V_y_CoM: {}, V_z_CoM: {}".format(self.v_com_pose[0,0], self.v_com_pose[0,1],
            #     self.v_com_pose[0,2]))

        # if debug:

        #     plt.figure(0)
        #     plt.plot(px)
        #     plt.plot(com_x)

        #     plt.figure(1)
        #     plt.plot(py)
        #     plt.plot(com_y)

        #     plt.figure(2)
        #     plt.plot(com_x,com_y)
        #     plt.plot(px, py)

        #     fig = plt.figure(3)
        #     ax = fig.add_subplot(111, projection='3d')
        #     ax.set_xlim3d(-0.1, 0.5)
        #     ax.set_ylim3d(-0.1, 0.5)
        #     ax.set_zlim3d(0, 0.2)

        #     com_trajectory = np.array(com_trajectory)
        #     l_trajectory = np.array(l_trajectory)
        #     r_trajectory = np.array(r_trajectory)

        #     plot_trajectory(ax=ax, P=com_trajectory, s=0.02, show_direction=False)
        #     plot_trajectory(ax=ax, P=r_trajectory,  s=0.02, show_direction=False)
        #     plot_trajectory(ax=ax, P=l_trajectory,  s=0.02, show_direction=False)

        #     plt.show()

def main():
    # FOOT_DISTANCE = 6.5 / 1000
    # COM_HEIGHT = 230 / 1000
    # X_OFFSET = 5 / 1000
    # COM_SWING = 115.5 / 1000
    # pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
    # pc.run()

    # forward kinematic test
    FOOT_DISTANCE = 6.5 / 1000
    COM_HEIGHT = 230 / 1000
    X_OFFSET = 5 / 1000
    COM_SWING = 115.5 / 1000
    pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
    ik = InverseKinematic()
    robot_model = createENOIDKinematicsModel()
    robot_model.forwardKinematics()

    time = 3
    init_time = 0
    while init_time < time:
        pc.update_walking_pattern()
        JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)
        robot_model.BODY.p = pc.com_pose.T
        robot_model.setJointAngle(JOINTS)
        robot_model.forwardKinematics()
        print(robot_model.L_LEG_J4.p[0])
        print(robot_model.R_LEG_J4.p[0])
        init_time += pc.dt


if __name__ == "__main__":
    main()