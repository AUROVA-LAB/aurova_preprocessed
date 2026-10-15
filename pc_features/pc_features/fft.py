# -*- coding: utf-8 -*-
"""
Created on Mon Nov 22 14:10:17 2021

@author: EPVelasco
"""

import numpy as np
import cv2

def fourier_filtter(imgI,mask,range):
 
    img = imgI/(2**8)                   ## 16 bits to 8 bits image
    img_float32 = np.float32(img)       ## int values to float values
    dft = cv2.dft(img_float32, flags = cv2.DFT_COMPLEX_OUTPUT)  ## discret fourier transform to de image
    dft_shift = np.fft.fftshift(dft)    #shift image 
    magnitude_spectrum = 20*np.log(cv2.magnitude(dft_shift[:,:,0],dft_shift[:,:,1]))    
    
    rows, cols = img.shape   
    fshift = dft_shift * mask           ## applied mask in dft image
    f_ishift = np.fft.ifftshift(fshift)
    img_mask = cv2.idft(f_ishift)       ## inverse dft
    img_mask = cv2.magnitude(img_mask[:,:,0],img_mask[:,:,1])
    img_mask = img_mask/2**16.0
    img_mask [img_mask<=range] = 0
    img_mask [img_mask>range] = 1

    mask_suelo = np.ones((rows, cols), np.uint8)   
    mask_suelo[int(rows/2):,:] = 0

    img_mask = np.logical_or(img_mask,mask_suelo)
    img_filter = imgI*img_mask  ## image filtered
    
    return(magnitude_spectrum , img_mask ,img_filter)
    
