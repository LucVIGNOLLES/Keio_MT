import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os
import time

from multiprocessing import Pool
from functools import partial
from random import randint, random, sample

from blend_keypoints import blend_func
from cam import Cam
from tsa import Tsa
from actuator import Actuator, bissect_lenght

# Global parameters
GAMMA0 = 0 # Start angle of the cam
GAMMAM = np.pi # End angle of the cam
STEP_SZ = 0.1 # Gamma angle step

GOAL = 20 # Goal reduction ratio

# Mutation parameters
GLOBAL_MUT_RATE = 0.5
GLOBAL_MUT_AMP = 0.2

LOCAL_MUT_RATE = 0.4
LOCAL_MUT_AMP = 0.1
LOCAL_MUT_NUM = 3

PERIM_MUT_RATE = 0.3
PERIM_MUT_AMP = 0.1


def evaluate_individual_force(indiv):
    """
    Method to be called in a multiprocessing pool 
    """
    return indiv.evaluate_force(GAMMA0, GAMMAM, STEP_SZ, GOAL, True)

def mutate_global(actuator, angle, amp):
    """
    Applies a uniform transformation on the whole cam, returns a list of keypoints
    """
    f = lambda an, am, o : am*np.cos(an - o) + 1
    new_kpts = [f(actuator.cam.step*i, amp, angle)*kpt for i, kpt in enumerate(actuator.cam.keypoints)]
    cam = Cam(new_kpts, actuator.cam.range, actuator.cam.perim)
    return Actuator(cam, actuator.tsa, actuator.xs, actuator.ys)

def mutate_hybrid(actuator, angle, amp, range):
    """
    Applies a smooth transformation on a specific part of the cam, returns a list of keypoints
    """
    f = lambda an, am, o, rng : am/rng*blend_func(an, o, rng) + 1
    new_kpts = [f(actuator.cam.step*i, amp, angle, range)*kpt for i, kpt in enumerate(actuator.cam.keypoints)]
    cam = Cam(new_kpts, actuator.cam.range, actuator.cam.perim)
    return Actuator(cam, actuator.tsa, actuator.xs, actuator.ys)

def mutate_perim(actuator, amp):
    new_perim = actuator.cam.perim + (random() - 0.5)*0.1*actuator.cam.perim
    cam = Cam(actuator.cam.keypoints, actuator.cam.range, new_perim)
    return Actuator(cam, actuator.tsa, actuator.xs, actuator.ys)

def mutate(actuator, scrore_mod_coeff):
    if random() < LOCAL_MUT_RATE:
        actuator = mutate_hybrid(actuator, np.pi*2*random(), scrore_mod_coeff*LOCAL_MUT_AMP, np.pi*random())

    for i in range(LOCAL_MUT_NUM):
        if random() < GLOBAL_MUT_RATE:
            actuator = mutate_global(actuator, np.pi*2*random(), scrore_mod_coeff*GLOBAL_MUT_AMP)

    if random() < PERIM_MUT_RATE:
        actuator = mutate_perim(actuator, scrore_mod_coeff*PERIM_MUT_AMP)

    return actuator

def mate(actu1, actu2):
    """
    Return an actuator that's randomly in between two other actuators in terms of perimeter and keypoints
    """
    # Perimeter
    a = random()
    new_perim = a*actu1.cam.perim + (1-a)*actu2.cam.perim

    # Keypoints
    r = random()
    new_keypoints = [r*kpt1 + (1-r)*kpt2 for kpt1, kpt2 in zip(actu1.cam.keypoints, actu2.cam.keypoints)]

    return Actuator(Cam(new_keypoints, 2, new_perim), actu1.tsa, actu1.xs, actu1.ys)


class Population:
    def __init__(self, population_sz, num_gen, known_cam = None) -> None:
        self.pop_size = population_sz # Will be cropped to the nearest lower 4 multiple in the repopulation process
        tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
        if known_cam is None:
            self.individuals = [Actuator.randomConvexCam(1, 2, 0.2, 0.5, tsa,  0.15, -0.2) for i in range(population_sz)]
        else:
            self.individuals = [mutate(Actuator(known_cam, tsa,  0.15, -0.2), 0.2) for i in range(population_sz)]
        self.scores = np.ones_like(self.individuals)
        self.all_gamma_lst = [[]]
        self.all_lh_lst = [[]]
        self.gen = 1
        self.max_gen = num_gen

        # Plot utilities

        self.fig = plt.figure(tight_layout=True)
        self.gs = gridspec.GridSpec(2, 2)

        self.ax1 = self.fig.add_subplot(self.gs[0,:])
        self.ax2 = self.fig.add_subplot(self.gs[1,0])
        self.ax3 = self.fig.add_subplot(self.gs[1,1])

        self.init_plot()

    def init_plot(self):
        plt.ion()

        self.ax1.set_xlim(1, self.max_gen)
        self.ax1.set_ylim(1,100000)
        self.ax1.set_yscale('log')
        
        # setting x-axis label and y-axis label
        self.ax1.set_xlabel("Generation")
        self.ax1.set_ylabel("Scores")

        self.ax2.set_xlim(-0.1, 0.1)
        self.ax2.set_ylim(-0.1,0.1)

        self.ax2.set_xlabel("Cam shape")
        self.ax2.set_aspect('equal')

        self.ax3.set_xlabel("Gamma")
        self.ax3.set_ylabel("Reduction ratio")
        
        self.ax3.set_xlim(0, np.pi)
        self.ax3.set_ylim(0, 120)

    def test(self):
        """
        Tests all the individuals at once using the Pool object form the multiprocessing module
        The evaluate_indiv function is called for that purpose
        """
        with Pool(self.pop_size) as p:
            pack = p.map(partial(evaluate_individual_force), self.individuals)

        self.scores = [s[0] for s in pack]
        self.all_gamma_lst = [s[1] for s in pack]
        self.all_lh_lst = [s[2] for s in pack]

    def select_and_repopulate(self, score_mod_coeff):
        """
        To be called only after scores have been computed.
        """

        # Sort the individuals by scores in increasing order and select the beginning of the list
        zip_sorted = zip(*sorted(zip(self.scores, self.individuals)))
        scores, individuals = list(zip_sorted)
        selected = list(individuals[:self.pop_size//2])

        # Take the best half of the selected individuals
        upper_class = selected[:self.pop_size//4]

        new_individuals = []

        # For each of them, select 2 partners and mate to repopulate. Random mutations are also applied
        for indiv in upper_class:
            for other_indiv in sample(selected, 2):
                new_indiv = mate(indiv, other_indiv)
                new_individuals.append(mutate(new_indiv, score_mod_coeff))


        # Keep the selected individuals and add the new ones
        self.individuals = selected + new_individuals

    def evolve(self):
        """
        Main method for Population class
        """
        # Folder for storing results
        str_current_datetime = time.strftime("%Y%m%d_%H%M%S")
        os.mkdir(os.getcwd() + '/results/'+ str_current_datetime)


        score_history = [[s] for s in self.scores]

        score_lines = [self.ax1.plot(range(self.gen), self.scores[i])[0] for i in range(self.pop_size)]

        X, Y = [], []
        for theta in np.arange(0, np.pi*2, 0.1):
            x, y = self.individuals[0].cam.r_cart(theta, 0)
            X.append(x)
            Y.append(y)

        cam_line, = self.ax2.plot(X, Y)
        center_pt, = self.ax2.plot(0,0, '.r')

        ref_line, = self.ax3.plot(0,0, linestyle='dashed')
        test_line, = self.ax3.plot(0,0)

        mod_coeff = 1

        for _ in range(self.max_gen):
            self.test()
            self.select_and_repopulate(mod_coeff)

            # Console callback
            print(">> gen : ", self.gen,"; best_score : ", min(self.scores))
            print(["%.2f" % x for x in self.scores])
            print(" ")

            # Plot update
            for i, score in enumerate(self.scores):
                score_history[i].append(score)
                score_lines[i].set_xdata(range(self.gen+1))
                score_lines[i].set_ydata(score_history[i])

            X, Y = [], []
        
            for theta in np.arange(0, np.pi*2, 0.1):
                x, y = self.individuals[0].cam.r_cart(theta, 0)
                X.append(x)
                Y.append(y)

            cam_line.set_xdata(X)
            cam_line.set_ydata(Y)

            self.ax2.set_adjustable('datalim')

            ref_line.set_xdata(self.all_gamma_lst[0])
            ref_line.set_ydata([20 for gamma in self.all_gamma_lst[0]])

            test_line.set_xdata(self.all_gamma_lst[0])
            test_line.set_ydata(self.all_lh_lst[0])
                
            self.fig.canvas.draw()
 
            # This will run the GUI event
            # loop until all UI events
            # currently waiting have been processed
            self.fig.canvas.flush_events()

            # Write fig
            self.fig.savefig('results/'+str_current_datetime+'/fig_gen_'+str(self.gen)+'.png')

            # Write to file
            with open('results/'+str_current_datetime+'/run.txt', 'a') as f:
                f.write('Generation ' + str(self.gen) + '\n')
                f.write('========' + '\n')
                for i, indiv in enumerate(self.individuals):
                    f.write(str(indiv.cam.keypoints) + ' ; ' + str(indiv.cam.perim) + ' ; ' + str(self.scores[i]) + '\n')
                f.write('========' + '\n\n')

            self.gen += 1
            mod_coeff = (1 - (self.gen/self.max_gen)**(1/2))*(min(self.scores)/5000)

        return self.individuals[0]    

if __name__ == "__main__":

    cam = Cam([0.01088736, 0.00902949, 0.00940026, 0.01240981, 0.01975054, 0.03287334,
                0.04894144, 0.06176267, 0.06186754, 0.04512916, 0.03005022, 0.01838828], 
                2, 
                0.21948481997605881)
    
    pop = Population(16, 150, cam)
    pop.evolve()

    plt.show()
