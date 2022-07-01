import numpy as np
from groundlight import Groundlight
import glob
import cv2
import matplotlib.pyplot as plt
import matplotlib
import os
import PIL
from PIL import Image
from io import BytesIO
import logging
np.random.seed(0)

def sweep_localize(gl, det, img, det_conf = 0.65, verbose = False):

    img_dims = np.array([img.shape[1],img.shape[0]])

    #Recursive Base Case, i.e. too many splits
    if img_dims[0] < 20 or img_dims[1] < 20:
        return None

    kernel_dims = (img_dims*2/3).astype(int)
    sweep_dims = (img_dims/4).astype(int)
    col_sweeps, row_sweeps = np.round(img_dims/sweep_dims).astype(int) - 1

    if verbose:
        fig, ax = plt.subplots(row_sweeps, col_sweeps, figsize = (16,12))

    slice_confidences = np.zeros((row_sweeps, col_sweeps))
    slice_dict = {}

    for col in range(col_sweeps):
        for row in range(row_sweeps):
            px_start = sweep_dims * np.array([col, row])
            pxEnd = px_start + kernel_dims

            slice_dict[(row, col)] = (px_start, pxEnd)
            slice_img_mat = img[px_start[1]:pxEnd[1],px_start[0]:pxEnd[0],:]

            image_query = mat_thru_det(gl, det, slice_img_mat)
            slice_label = image_query.result.label
            slice_conf = image_query.result.confidence

            if slice_label == 'PASS' and slice_conf > det_conf:
                if verbose:
                    ax[row, col].set_title('cube found, conf: '+ str(np.round(slice_conf,2)))
                    ax[row, col].imshow(slice_img_mat)
                slice_confidences[row, col] = slice_conf
            else:
                if verbose:
                    ax[row, col].imshow(cv2.cvtColor(slice_img_mat,cv2.COLOR_BGR2GRAY), cmap = 'ocean')
                slice_confidences[row, col] = 0

    max_1d_ind = np.argmax(slice_confidences)
    max_2d_ind = np.unravel_index(max_1d_ind, slice_confidences.shape)

    best_conf = slice_confidences[max_2d_ind]
    if best_conf < 0.65:
        return None

    if verbose:
        ax[max_2d_ind[0], max_2d_ind[1]].set_title(
            'CUBE FOUND, BEST CONF: ' + str(np.round(best_conf,2)), fontsize = 15, color = 'lime')
        plt.show()

    best_px_start, best_px_end = slice_dict[max_2d_ind]
    best_slice_img_mat = img[best_px_start[1]:best_px_end[1],best_px_start[0]:best_px_end[0],:]

    return (best_px_start,
            np.minimum(best_px_end, img_dims),
            sweep_localize(gl, det, best_slice_img_mat, det_conf = det_conf, verbose = verbose))

def mat_thru_det(gl, det, mat):
    img_PIL = Image.fromarray(cv2.cvtColor(mat, cv2.COLOR_BGR2RGB))
    byte_io = BytesIO()

    img_PIL.save(byte_io, 'jpeg')
    jpg_buffer = byte_io.getvalue()
    byte_io.close()

    image_query = gl.submit_image_query(detector_id=det.id, image = jpg_buffer)

    return image_query

def assemble_px_tree(tree):
    return assemble_px_tree_helper(np.array([0,0]), tree)

def assemble_px_tree_helper(oldPx, tree):
    newPx = oldPx + tree[0]
    if(tree[2]) is None:
        newPx += ((tree[1] - tree[0])/2).astype(int)
        return newPx
    return assemble_px_tree_helper(newPx, tree[2])


def plot_tree_on_image(ax, tree, img):
    ax.imshow(img)
    plot_tree_on_image_helper(ax, tree, np.array([0,0]))
    return ax

def plot_tree_on_image_helper(ax, tree, prevPx):

    cmap = matplotlib.cm.get_cmap('rainbow')

    if tree[2] is None:
        center = prevPx + ((tree[0] + tree[1])/2).astype(int)
        ax.scatter([center[0]], [center[1]], marker = 'x', s = 60, color = 'black')
        return

    color = cmap(np.random.rand())
    ax.scatter([tree[0][0] + prevPx[0]],[tree[0][1] + prevPx[1]],marker = 'o', s = 30, color = color)
    ax.scatter([tree[1][0] + prevPx[0]],[tree[1][1] + prevPx[1]],marker = 'o', s = 30, color = color)
    prevPx += tree[0]
    plot_tree_on_image_helper(ax, tree[2], prevPx)
