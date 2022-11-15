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

def evaluate_individual_force(gamma0, gammam, step_sz, goal_coeff, indiv):
    """
    Method to be called in a multiprocessing pool 
    """
    return indiv.evaluate_force(gamma0, gammam, step_sz, goal_coeff, True)

def mutate_keypoint(actuator, kpt_idx, amp):
    if kpt_idx==0:
        b = bissect_lenght(actuator.cam.keypoints[len(actuator.cam.keypoints)-1], actuator.cam.keypoints[1], 2*actuator.cam.step)
    elif kpt_idx==len(actuator.cam.keypoints)-1:
        b = bissect_lenght(actuator.cam.keypoints[len(actuator.cam.keypoints)-2], actuator.cam.keypoints[0], 2*actuator.cam.step)
    else:
        b = bissect_lenght(actuator.cam.keypoints[kpt_idx-1], actuator.cam.keypoints[kpt_idx+1], 2*actuator.cam.step)

    # Performing mutation by taking into account the bissecting lenght 
    if actuator.cam.keypoints[kpt_idx] < b:
        new_kpt = actuator.cam.keypoints[kpt_idx] + random()*amp*actuator.cam.keypoints[kpt_idx]
    else:
        new_kpt = actuator.cam.keypoints[kpt_idx] - random()*amp*actuator.cam.keypoints[kpt_idx]

    return new_kpt

def mutate_global(actuator, angle, amp):
    f = lambda an, am, o : am*np.cos(an - o) +1
    return [f(actuator.cam.step*i, amp, angle)*kpt for i, kpt in enumerate(actuator.cam.keypoints)]

def mutate_hybrid(actuator, angle, amp, range):
    f = lambda an, am, o, rng : am/rng*blend_func(an, o, rng) + 1
    return [f(actuator.cam.step*i, amp, angle, range)*kpt for i, kpt in enumerate(actuator.cam.keypoints)]


def mutate(actuator, kpt_rate, kpt_amp, perim_rate, global_rate, global_amp):
    """
    Returns a mutated version of the actuator given as an argument.
    Mutations have a random chance to happen, according to the mutation rates.
    Mutations can modify a keypoint, by a random amount between 0 and 7% of current lenght, in the direction of making the cam more convex
    Mutations ca also happen on the perimeter, between 0 and 10% of its current value
    """
    # TODO: Add arguments for selecting the mutation magnitude
    # TODO: Add a mutation range, so that a mutation can affect more than one keypoint at a time, to facilitate bigger changes in radius in parts of the cam
    # TODO: Add a mutation that shifts the position of the center of rotation of the cam (or the equivalent operation on the keypoints) 
    #     |_> way too hard to compute, we better stick to global an uniform transormations of the keypoints to stay effective

    # Mutate keypoints
    # new_kpts = []
    # for i, kpt in enumerate(actuator.cam.keypoints):
    #     if random() < kpt_rate:
    #         new_kpts.append(mutate_keypoint(actuator, i, kpt_amp))
    #     else:
    #         new_kpts.append(kpt)

    new_kpts = actuator.cam.keypoints

    if random() < kpt_rate:
        new_kpts = mutate_hybrid(actuator, np.pi*2*random(), kpt_amp, np.pi*random())

    # Mutate perimeter
    if random() < perim_rate:
        new_perim = actuator.cam.perim + (random() - 0.5)*0.1*actuator.cam.perim
    else:
        new_perim = actuator.cam.perim

    # Mutate globally (will overwrite individual mutations)
    if random() < global_rate:
        new_kpts = mutate_global(actuator, np.pi*2*random(), global_amp)
    
    # Create mutated cam and return corresponding actuator
    cam = Cam(new_kpts, actuator.cam.range, new_perim)
    return Actuator(cam, actuator.tsa, actuator.xs, actuator.ys)

def tmp(actuator, angle, amp, range):
    new_kpts = mutate_hybrid(actuator, angle, amp, range)
    cam = Cam(new_kpts, actuator.cam.range, actu.cam.perim)
    return Actuator(cam, actuator.tsa, actuator.xs, actuator.ys)


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
    def __init__(self, population_sz) -> None:
        self.pop_size = population_sz # Will be cropped to the nearest lower 4 multiple in the repopulation process
        tsa = Tsa(0.2, 0.003, 0.01, 0, 1)
        self.individuals = [Actuator.randomConvexCam(1, 2, 0.2, 0.5, tsa,  0.15, -0.2) for i in range(population_sz)]
        self.scores = np.zeros_like(self.individuals)
        self.all_gamma_lst = [[]]
        self.all_lh_lst = [[]]
        self.gen = 1

    #TODO: Add a method to specify a base individual and heavily mutate it to generate the base population 
    # (better if we already have a good candidate)

    def test(self):
        """
        Tests all the individuals at once using the Pool object form the multiprocessing module
        The evaluate_indiv function is called for that purpose
        """
        with Pool(self.pop_size) as p:
            pack = p.map(partial(evaluate_individual_force, 0, np.pi, 0.1, 20), self.individuals)

        self.scores = [s[0] for s in pack]
        self.all_gamma_lst = [s[1] for s in pack]
        self.all_lh_lst = [s[2] for s in pack]

    def select_and_repopulate(self, kpt_mutation_rate, perim_mut_rate):
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
                new_individuals.append(mutate(new_indiv, kpt_mutation_rate, 0.2, perim_mut_rate, 0.2, 0.2))


        # Keep the selected individuals and add the new ones
        self.individuals = selected + new_individuals

    def evolve(self, num_gen, kpt_mutation_rate, perim_mut_rate):
        """
        Main method for Population class
        """
        str_current_datetime = time.strftime("%Y%m%d_%H%M%S")
        os.mkdir(os.getcwd() + '/results/'+ str_current_datetime)

        score_history = [[s] for s in self.scores]

        # to run GUI event loop
        plt.ion()
        
        # here we are creating sub plots
        fig = plt.figure(tight_layout=True)
        gs = gridspec.GridSpec(2, 2)

        ax1 = fig.add_subplot(gs[0,:])

        ax1.set_xlim(1, num_gen)
        ax1.set_ylim(0,100000)
        lines = [ax1.plot(range(self.gen), self.scores[i])[0] for i in range(self.pop_size)]
        
        # setting title
        plt.title("Cam evolution", fontsize=20)
        
        # setting x-axis label and y-axis label
        ax1.set_xlabel("Generation")
        ax1.set_ylabel("Scores")

        X, Y = [], []
        
        ax2 = fig.add_subplot(gs[1,0])
        for theta in np.arange(0, np.pi*2, 0.1):
            x, y = self.individuals[0].cam.r_cart(theta, 0)
            X.append(x)
            Y.append(y)

        cam_line, = ax2.plot(X, Y)
        center_pt, = ax2.plot(0,0, '.r')

        ax3 = fig.add_subplot(gs[1,1])

        ax3.set_xlim(0, np.pi)
        ax3.set_ylim(0, 120)

        ref_line, = ax3.plot(0,0, linestyle='dashed')
        test_line, = ax3.plot(0,0)

        for _ in range(num_gen):
            self.test()
            self.select_and_repopulate(kpt_mutation_rate, perim_mut_rate)
            print(">> gen : ", self.gen,"; best_score : ", min(self.scores))
            print(["%.2f" % x for x in self.scores])
            print(" ")

            for i, score in enumerate(self.scores):
                score_history[i].append(score)
                lines[i].set_xdata(range(self.gen+1))
                lines[i].set_ydata(score_history[i])

            X, Y = [], []
        
            for theta in np.arange(0, np.pi*2, 0.1):
                x, y = self.individuals[0].cam.r_cart(theta, 0)
                X.append(x)
                Y.append(y)

            cam_line.set_xdata(X)
            cam_line.set_ydata(Y)

            ax2.set_aspect('equal')

            ref_line.set_xdata(self.all_gamma_lst[0])
            ref_line.set_ydata([20 for gamma in self.all_gamma_lst[0]])

            test_line.set_xdata(self.all_gamma_lst[0])
            test_line.set_ydata(self.all_lh_lst[0])
                
            fig.canvas.draw()
 
            # This will run the GUI event
            # loop until all UI events
            # currently waiting have been processed
            fig.canvas.flush_events()

            fig.savefig('results/'+str_current_datetime+'/fig_gen_'+str(self.gen)+'.png')

            # Write to file

            with open('results/'+str_current_datetime+'/run.txt', 'a') as f:
                f.write('Generation ' + str(self.gen) + '\n')
                f.write('========' + '\n')
                for i, indiv in enumerate(self.individuals):
                    f.write(str(indiv.cam.keypoints) + ' ; ' + str(indiv.cam.perim) + ' ; ' + str(self.scores[i]) + '\n')
                f.write('========' + '\n\n')

            self.gen += 1

        return self.individuals[0]

# Testing ======

GAMMA0 = 0
GAMMAM = np.pi

if __name__ == "__main__":
    pop = Population(32) # Size must be dividable by 4

    actu = pop.evolve(100, 0.4, 0.2)

    plt.show() 

    