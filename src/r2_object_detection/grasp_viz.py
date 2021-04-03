import cv2
import numpy as np
#from builtins import int, len, range, list, float, sorted, max, min
import math


def calc_pred_rect(img, points, gripper_h):
    """returns array of corner points, polygon of points, and angle"""
    # ================ edge detected grasp point =================
    # gripper_h is taken from labels, just to maintain consistency
    # don't know the specs of the end effector yet so
    x1, x2 = points[0], points[2]
    y1, y2 = points[1], points[3]
    diff_x = abs(x1 - x2)
    diff_y = abs(y1 - y2)
    pred_angle = math.atan2(y2-y1, x2-x1)  # in radians

    # TODO: Is my trig correct? T_T

    gripper_w_pred = math.sqrt(diff_x ** 2 + diff_y ** 2) / 2
    dx_w_pred = gripper_w_pred * math.cos(pred_angle)
    dy_w_pred = gripper_w_pred * math.sin(pred_angle)

    dx_h_pred = math.sqrt(gripper_h ** 2 + gripper_w_pred **
                          2) * math.sin(pred_angle)
    dy_h_pred = math.sqrt(gripper_h ** 2 + gripper_w_pred **
                          2) * math.cos(pred_angle)

    # recalculate center by taking average of two points' components
    ctr_x_pred = (x1 + x2) / 2
    ctr_y_pred = (y1 + y2) / 2

    # draw the two gripper dots again (just in case they got covered)
    cv2.circle(img, (int(x1), int(y1)),
               5, (255, 0, 255), -1)
    cv2.circle(img, (int(x2), int(y2)),
               5, (255, 0, 255), -1)

    prect = np.array([[ctr_x_pred - dx_w_pred - dx_h_pred, ctr_y_pred - dy_w_pred + dy_h_pred],
                      [ctr_x_pred - dx_w_pred + dx_h_pred,
                       ctr_y_pred - dy_w_pred - dy_h_pred],
                      [ctr_x_pred + dx_w_pred + dx_h_pred,
                       ctr_y_pred + dy_w_pred - dy_h_pred],
                      [ctr_x_pred + dx_w_pred - dx_h_pred, ctr_y_pred + dy_w_pred + dy_h_pred]], np.int32)

    # predictions = Polygon([(ctr_x_pred - dx_w_pred - dx_h_pred, ctr_y_pred - dy_w_pred + dy_h_pred),
    #                        (ctr_x_pred - dx_w_pred + dx_h_pred,
    #                         ctr_y_pred - dy_w_pred - dy_h_pred),
    #                        (ctr_x_pred + dx_w_pred + dx_h_pred,
    #                         ctr_y_pred + dy_w_pred - dy_h_pred),
    #                        (ctr_x_pred + dx_w_pred - dx_h_pred, ctr_y_pred + dy_w_pred + dy_h_pred)])

    # prect for plotting
    return prect


def plot_pred_rect(img, prect):
    """input must be an image and nparray of corner points"""
    cv2.polylines(img, [prect], True, (0, 255, 0), 1)
    # cv2.imshow("pred labels", img)
    # cv2.waitKey(0)

