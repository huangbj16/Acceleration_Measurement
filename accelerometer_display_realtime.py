#!/usr/bin/env python
# encoding: utf-8

import numpy as np
import matplotlib.pyplot as plt
import json
import time
from multiprocessing import Process
from serial_control import serial_control

class AccelerationAnalyzer:

    def __init__(self, port):
        self.control = serial_control(port)
        # Main 
        self.all_data = []
        self.all_data_timestamps = []
        self.buffer = []
        self.buffer_size = 500
        self.buffer_fft = []
        self.buffer_fft_size = 4000
        self.buffer_timestamps = []
        self.frame_count = 0
        self.baseline_acc = None
        self.is_baseline_valid = False

        # data_thread = threading.Thread(target=self.receive_data)
        # data_thread.start()

        # self.receive_data()

        self.loop()

    def loop(self):
        try:
            self.last_time = time.time()
            while True :
                # self.last_time = 1
                # cur_time = time.time()
                # if cur_time - self.last_time > 1.0:
                #     self.last_time = cur_time
                #     print('elapsed! buffer = ', len(self.buffer_fft), '! time = ', self.last_time)
                    # self.freq_plot()
                    # self.buffer_fft.clear()

                self.receive_data()
                self.frame_count = self.frame_count + 1
                if self.frame_count == 100:
                    if not self.is_baseline_valid:
                        self.baseline_calculate()
                    self.frame_count = 0
                    self.graph_plot()

        except KeyboardInterrupt:
            self.control.finish()
            print(self.frame_count)
            f = open('acc_data.txt', 'w')
            self.all_data_np = np.array(self.all_data)
            for i in range(len(self.all_data)):
                f.write(json.dumps({'time': self.all_data_timestamps[i], 'acc': np.round((self.all_data[i] - self.baseline_acc), 3).tolist()})+"\n")
            f.close()
            print("data saved!")
        print("End...")

    def receive_data(self):
        try:
            self.input = self.control.receive()
            # print(len(self.input), self.input)
            self.processing_data(self.input)
        except Exception as e:
            print('data error = ', e)

    def processing_data(self, input_bytes):
        # print(input_bytes)
        input_string = str(input_bytes, encoding='utf8')
        try:
            input_string = input_string[:input_string.index('}')+1]
            data = json.loads(input_string)
            timestamp = data['T']/1000
            data = [data['X'], data['Y'], data['Z']]
            self.all_data.append(data)
            self.all_data_timestamps.append(timestamp)
            self.buffer.append(data)
            self.buffer_timestamps.append(timestamp)
            if len(self.buffer) > self.buffer_size:
                self.buffer.pop(0)
                self.buffer_timestamps.pop(0)
            # self.buffer_fft.append(data)
        except:
            print('error data = ', input_bytes, '')

    def baseline_calculate(self):
        acc_array = np.array(self.buffer)
        self.baseline_acc = np.mean(acc_array, axis=0)
        print(self.baseline_acc)
        self.is_baseline_valid = True
        # self.baseline_acc = acc_rms_axis

    def freq_plot(self):
        plt.clf()
        
        acc_array = np.array(self.buffer_fft)
        acc_len = acc_array.shape[0]
        # print(acc_array.shape)
        acc_abs = np.linalg.norm(acc_array, axis=1)
        # acc_abs = [np.sqrt(acc_array[i][0]*acc_array[i][0] + acc_array[i][1]*acc_array[i][1] + acc_array[i][2]*acc_array[i][2]) for i in range(acc_array.shape[0])]
        # print(acc_abs.shape)

        acc_fft = np.fft.fft(acc_abs)[1:acc_len>>1]
        acc_freq = np.fft.fftfreq(acc_len, 1/acc_len)[1:acc_len>>1]

        plt.plot(acc_freq, 2.0 / acc_len * np.abs(acc_fft))
        plt.xlim(0, 400)
        plt.ylim(0, 60)
        plt.xticks(np.arange(0, 400, 20))
        plt.pause(0.001)

    def graph_plot(self):
        plt.clf() # clear previous figure

        acc_array = np.array(self.buffer)

        # print(type(self.baseline_acc))
        if type(self.baseline_acc) == type(acc_array):
            for i in range(acc_array.shape[0]):
                acc_array[i] = np.round((acc_array[i] - self.baseline_acc), 2)

        # print(acc_array.shape)
        acc_abs = np.linalg.norm(acc_array, axis=1)
        acc_rms = np.round(np.sqrt(np.mean(acc_abs**2)), 2) 
        acc_rms_axis = np.sqrt(np.mean(acc_array**2, axis=0))
        print(acc_rms_axis, acc_rms)

        # wave
        plt.subplot(211)
        x = np.arange(acc_array.shape[0])
        plt.plot(x, acc_array[:, 0], x, acc_array[:, 1], x, acc_array[:, 2],)
        plt.legend(['x', 'y', 'z'])
        plt.axis([0, self.buffer_size, -50, 50])
        # plt.xlabel("time")
        plt.ylabel("acceleration")
        display_text = 'MAX X='+str(np.max(acc_array[:, 0]))+' Y='+str(np.max(acc_array[:, 1]))+' Z='+str(np.max(acc_array[:, 2]))+'\n'+\
                       'MIN X='+str(np.min(acc_array[:, 0]))+' Y='+str(np.min(acc_array[:, 1]))+' Z='+str(np.min(acc_array[:, 2]))
        plt.title(display_text)
        #Spectrum
        plt.subplot(212)
        plt.plot(x, acc_abs)
        plt.axis([0, self.buffer_size, 0, 100])
        plt.xlabel("frame")
        plt.ylabel("acceleration")
        display_text = 'Max ACC = '+str(acc_rms)
        plt.title(display_text)
        #Pause
        plt.pause(.001)


if __name__ == "__main__":
    spec = AccelerationAnalyzer(port='COM16')
    