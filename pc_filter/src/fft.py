# -*- coding: utf-8 -*-
"""
Created on Mon Nov 22 14:10:17 2021

@author: EPVelasco
"""

import numpy as np
import cv2

def fourier_filtter(imgI,mask,range):
 
    img = imgI/(2**8)
    #img = imgI
    img_float32 = np.float32(img)
    dft = cv2.dft(img_float32, flags = cv2.DFT_COMPLEX_OUTPUT)
    dft_shift = np.fft.fftshift(dft)
    magnitude_spectrum = 20*np.log(cv2.magnitude(dft_shift[:,:,0],dft_shift[:,:,1]))
    
    # mask
    rows, cols = img.shape    
   # limits_cols = int(round(cols/2))
   # limits_rows = int(round(rows/2))
   # mask = 0*np.ones((rows, cols, 2), np.uint8)
   
    #mask[limits_rows:limits_rows+1,:] = 1
    #mask[:, limits_cols-1:limits_cols+1] = 0

    #mask_2 =0* np.ones((rows, cols), np.uint8)
    #mask_2[:, limits_cols:limits_cols+1] = 1
    #mask_2[limits_rows-1:limits_rows+1,:] = 0
  
    fshift = dft_shift * mask
    f_ishift = np.fft.ifftshift(fshift)
    img_back = cv2.idft(f_ishift)
    img_back = cv2.magnitude(img_back[:,:,0],img_back[:,:,1])
    img_back_2 = img_back/2**16.0
    img_back = img_back/2**16.0
    #hist = cv2.calcHist([img_back], [0], None, [256], [0, 256])
    #img_filter = img_back.copy()
    #range = 20
    img_back [img_back<=range] = 0
    img_back [img_back>range] = 1

    mask_suelo = np.ones((rows, cols), np.uint8)   
    mask_suelo[int(rows/2):,:] = 0

    img_back = np.logical_or(img_back,mask_suelo)

    #img_back_2 [img_back_2<=range] = 0
    #img_back_2 [img_back_2>range] = 1


    img_filter = imgI*img_back
    #img_filter_inv = img*img_back 
    
    return(magnitude_spectrum , img_back_2 ,img_filter)
    #return(img_filter)
    
