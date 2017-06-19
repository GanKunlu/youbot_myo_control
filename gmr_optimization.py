# -*- coding: gbk -*-
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import *
import seaborn as sns
from sklearn.utils import check_random_state
from gmr import GMM, plot_error_ellipses

class gmr_from_file(object):
    def __init__(self,n_components,file_name):
        self.random_state = check_random_state(3)
        self.n_components = n_components
        self.file_name = file_name
        self.angle_lr = []
        self.angle_ud = []
        self.angle_roll = []
        self.grriper = []
        self._load_data()
        self.size = len(self.angle_lr)
        self.opti_data = np.empty((self.size,2))
        self.color = ["#4C72Bf","#8172B2","#CCB974","#64B5CD","#A38CF4","#F461DD"]
        sns.set_style("whitegrid")

    def _load_data(self):
        with open(self.file_name, "r") as file:
            for line in file.readlines():
                list_from_line = line.strip().split(" ")
                self.angle_lr.append(float(list_from_line[0]))
                self.angle_ud.append(float(list_from_line[1]))
                self.angle_roll.append(float(list_from_line[4]))
                self.grriper.append(float(list_from_line[5]))

    def _estimate_error(self, ellipses):
        k_err = []
        for mean, (angle, width, height) in ellipses:
            k_err.append(height*width)
        return sum(k_err)

    def do_gmr(self,is_plot = False, n_iter= 1):
        print("Doing GMR optimization...")
        X_test = np.linspace(0, self.size*0.01,self.size)
        X1 = np.vstack((X_test, np.array(self.angle_lr))).T
        X2 = np.vstack((X_test, np.array(self.angle_ud))).T
        err_list1 = []
        err_list2 = []

        # select n_components with n_iter steps:
        for i in range(n_iter):
            try:
                gmm1 = GMM(n_components=self.n_components[0]+i, random_state=1)
                gmm1.from_samples(X1)
                err_list1.append(self._estimate_error(gmm1.to_ellipses()))

            except:
                err_list1.append(100)

            try:
                gmm2 = GMM(n_components=self.n_components[1]+i, random_state=1)
                gmm2.from_samples(X2)
                err_list2.append(self._estimate_error(gmm2.to_ellipses()))
            except:
                err_list2.append(100)

        select_n = [self.n_components[0] + err_list1.index(min(err_list1)),
                    self.n_components[1] + err_list2.index(min(err_list2))]
        print("Select n_components is "+str(select_n))

        gmm_1 = GMM(n_components=select_n[0], random_state=1)
        gmm_1.from_samples(X1)
        gmm_2 = GMM(n_components=select_n[1], random_state=1)
        gmm_2.from_samples(X2)

        Y1 = gmm_1.predict(np.array([0]), X_test[:, np.newaxis])
        Y2 = gmm_2.predict(np.array([0]), X_test[:, np.newaxis])

        self.opti_data[:, 0] = Y1.T
        self.opti_data[:, 1] = Y2.T
        ori_data = np.array([self.angle_lr,self.angle_ud])
        gmms = [gmm_1,gmm_2]
        if (is_plot):
            myfront = FontProperties(fname="/usr/share/fonts/truetype/wqy/wqy-microhei.ttc")
            for i in range(2):
                plt.subplot(2, 1, i+1)
                #plt.title("Linear: $p(Y | X) = \mathcal{N}(\mu_{Y|X}, \Sigma_{Y|X})$")
                plt.scatter(X_test, ori_data[i], c = self.color[0], label = "oringinal",alpha= 0.8)
                plot_error_ellipses(plt.gca(), gmms[i], colors = self.color[1:])
                plt.plot(X_test,  self.opti_data[:, i], c = "k", lw = 1.7, label = "GMR")
                plt.xlim((0,self.size*0.01))
                plt.xlabel(u"时间/$s$", fontproperties = myfront)
                plt.ylabel(u"关节位置/$rad$", fontproperties = myfront)
                plt.legend()
            plt.show()

    def protect_wrist(self, angle_move):
        angle = angle_move
        # 1.7
        if angle_move >= 1.7:
            angle = 1.7
        if angle_move < 0:
            angle = 0.01
        return angle

    def save_to_file(self):
        with open("demon_file1.txt","w") as file:
            print("saving optimized data...")
            for i, it in enumerate(self.opti_data):
                angle1 = it[0]
                angle2 = it[1]
                if(self.angle_ud[i] < 0.8236):
                    angle1 = self.angle_lr[i]
                    angle2 = self.angle_ud[i]
                angleUD = (angle2 - 1.0) / 0.0036
                angle3 = (angleUD + 90) * 0.015889 - 4.5
                angle4 = self.protect_wrist(0.8 + (angleUD+ 90) * 0.003)
                angle5 = self.angle_roll[i]
                grrip_position = self.grriper[i]
                file.write("{0} {1} {2} {3} {4} {5} {6}\n".format(angle1, angle2, angle3, angle4, angle5, grrip_position, grrip_position))
        print("done! save as: "+self.file_name)




'''example:
dem_opter = gmr_from_file([9,6],"demon_file.txt")
dem_opter.do_gmr(is_plot = True, n_iter=5)
#dem_opter.save_to_file()
'''



