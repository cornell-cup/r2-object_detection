"""A class used to test the functionality of our other kinematics classes.

Written by Yashraj Sinha '25 and Simon Kapen '24, Spring 2022.
"""
import random

from matplotlib import pyplot as plt
from matplotlib.widgets import Button
import optimizers
import collision_detection
import arm_node


def test_series_optimizer():
    """
        This method will test the series optimizer from the optimizers class
    :return:
    """
    """
        Places a prism randomly in 3D space, and draws a larger prism around it that represents it's
        bounding box. Then checks if a randomly generated arm collides the outer prism at any point.

        """
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # Set the limits for the range of plotted values
    lim = .6
    plt.xlim(-lim, lim)
    plt.ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    # create the prism to be avoided (hard coded)
    lim = .3
    location = random.uniform(-lim, lim)
    prism = ([location, 0, 0, .2, .3, .4])
    exoPrism = ([prism[0] - .05, prism[1] - .05, prism[2] - .05, prism[3] + .1, prism[4] + .1, prism[5] + .1])
    collision_detection.plot_linear_prism(ax, prism, 'blue')
    collision_detection.plot_linear_prism(ax, exoPrism, 'red')
    lim = .5
    """
    This code was used to generate a single line, and has since been replaced 
    by code that generates a random N link arm. 

        randomX1 = random.uniform(-lim, lim)
        randomY1 = random.uniform(-lim, lim)
        randomZ1 = random.uniform(-lim, lim)

        randomX2 = random.uniform(-lim, lim)
        randomY2 = random.uniform(-lim, lim)
        randomZ2 = random.uniform(-lim, lim)
        line_seg = ([randomX1, randomY1, randomZ1,
                     randomX2, randomY2, randomZ2])  # ([-.1, .1, .1, .3, .1, .1])
        # line_seg = ([-.3,.15,-.2,.6,.15,.2]) # intersects x faces
        # line_seg = ([.16, -.3, .2, .16, .3, .2]) # intersects y faces
        # line_seg = ([.2, -.2, -.3, .2, .15, .5]) # intersects z faces
        xs = ([line_seg[0], line_seg[3]])
        ys = ([line_seg[1], line_seg[4]])
        zs = ([line_seg[2], line_seg[5]])
        plot_lines(ax, xs, ys, zs, None)
    """
    # Create a random arm
    arm = arm_node.Node(None)
    print(arm)
    collision_detection.plot_arm(ax, arm, None)
    #optimizers.optimize(arm,exoPrism)
    class Index:
        def draw(self, event):
            plt.close()
            test_series_optimizer()

    axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
    run = Button(axnext, 'ReRun')
    callback = Index()
    run.on_clicked(callback.draw)
    if collision_detection.checkCollision(exoPrism, arm):
        plt.title("Collision Detected!")
    else:
        plt.title("Wow no Collision")
    plt.show()


if __name__ == "__main__":
    test_series_optimizer()
