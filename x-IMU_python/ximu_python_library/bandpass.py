# -*- coding: utf-8 -*-
"""
Created on Tue Mar  8 09:36:47 2016

@author: Scott
"""

from scipy.signal import butter, lfilter, bode
import scipy
import math

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

'''
if __name__ == "__main__":
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.signal import freqz
    import csv
    
    # read hr from csv
    f = open('./463992791_Json_to_CSV.csv', 'r')  # Scott Interval
    # f = open('./451994225_Json_to_CSV.csv', 'r')  # Scott 10km HR spike
    # f = open('./477001513_Json_to_CSV.csv', 'r')  # Scott 32km
    # f = open('./455389798_Json_to_CSV.csv', 'r')  # Scott 16km
    # f = open('./454266577_Json_to_CSV.csv', 'r')  # Scott 5km interval
    # f = open('./473740959_Json_to_CSV.csv', 'r')  # trulyman
    w_hr = []
    w_t = []
    i = 0
    for row in csv.DictReader(f):  
    # print(row['heartrate'])
    # auto padding if hr <= 0

        if len(w_t) == 0 or (int(row['w_t']) > w_t[len(w_t)-1]):
            if int(row['w_hr']) <= 0:
                w_hr += [w_hr[len(w_hr) - 1]]
            else:    
                w_hr += [int(row['w_hr'])]
                       
            w_t += [i]
            i += 1
    
    f.close() 
    print('max w_t[]:{}'.format(w_t[len(w_t)-1]))


    # Sample rate and desired cutoff frequencies (in Hz).
    fs = 1.0  #  1.0/5.0 for trulyman due to down sampling
    lowcut = 1/300.0
    highcut = 1/10.0

    # Plot the frequency response for a few different orders.
    plt.figure(1)
    plt.clf()
    for order in [3]:
        B, A = butter_bandpass(lowcut, highcut, fs, order=order)
        w, h = freqz(B, A, worN=2000)
        plt.plot((fs * 0.5 / np.pi) * w, abs(h), label="order = %d" % order)

    w, mag, phase = bode((B, A))
    # print("w:{}, mag:{}, phase:{}".format(w, mag, phase))    
    plt.plot([0, 0.5 * fs], [np.sqrt(0.5), np.sqrt(0.5)],
             '--', label='sqrt(0.5)')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain')
    plt.xlim([0,0.1])
    plt.grid(True)
    plt.legend(loc='best')
        
    
    a = w_hr
    t = w_t  
    b = butter_bandpass_filter(a, lowcut, highcut, fs, order=3)
       
    # implementation of butter filter
    b2 = []
    for i in range(len(a)):
        if i <= 6:
            output = a[i]            
        else:        
            output = (
                     -1 * (
                            b[i-1] * -4.7679276
                          + b[i-2] * 9.5196287
                          + b[i-3] * -10.2357500
                          + b[i-4] * 6.2751150
                          + b[i-5] * -2.0817284
                          + b[i-6] * 0.2906636
                          )
                    + 1 * (
                            a[i] * 0.0166004
                          + a[i-1] * 0.0
                          + a[i-2] * -0.0498012
                          + a[i-3] * 0.0
                          + a[i-4] * 0.0498012
                          + a[i-5] * 0.0
                          + a[i-6] * -0.0166004
                          )
                     )
            b2 += [output]

    # signal cut to remove ~250 phase delay
    phase_delay = 110 #30 for trulyman  #150 for scott
    acut = a[0:-(phase_delay+1)]
    bcut = b[phase_delay:-1]
    ccut = acut - bcut
    tcut = w_t[0:-(phase_delay+1)]
    
    plt.figure(2)
    plt.clf()
    plt.plot(tcut, acut, label='Noisy signal')
    plt.plot(tcut, bcut)
    plt.plot(tcut, ccut)    
    
    plt.xlabel('time (seconds)')
    plt.grid(True)
    plt.axis('tight')
    plt.ylim([-50,200])
    # plt.legend(loc='upper left')
    
    af = scipy.fftpack.fft(acut)
    plt.figure(3)
    plt.clf()
    plt.plot(abs(af))
    plt.ylim([0,5000])
    plt.xlim([0, 100])
    
    bf = scipy.fftpack.fft(bcut)
    plt.figure(4)
    plt.clf()
    plt.plot(abs(bf))
    plt.ylim([0,5000])
    plt.xlim([0, 100])

    f1 = 0.5 #Hz
    f2 = 3 #Hz
    sr = 10.0 #sample rate Hz >= 2*f
    x = np.arange(0,100,1/sr)
    test = np.sin(2*np.pi*f1*x) + np.sin(2*np.pi*f2*x)*0.5
    testf = scipy.fftpack.fft(test)
    plt.figure(5)
    plt.clf()
    plt.plot(x[0:len(x)/2]/len(x)*sr**2,abs(testf[0:len(x)/2]))    
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amp')

    plt.show()
    
    astd = np.std(acut)
    avar = np.var(acut)    
    
    bstd = np.std(bcut)
    bvar = np.var(bcut)
    
    cstd = np.std(ccut)
    cvar = np.var(ccut)
    print("avar:{}, bvar:{}, cvar:{}".format(avar, bvar, cvar))
    print("astd:{}, bstd:{}, cstd:{}".format(astd, bstd, cstd))
'''