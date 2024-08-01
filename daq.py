from pyftdi import gpio as gp
import time
import threading
import numpy as np
import matplotlib.pyplot as plt

SAMPLE_FREQ = int(1e5)
SPI_RATE = int(4e3)
SPI_LENGTH = 24
SPI_WINDOW = int(SPI_LENGTH / SPI_RATE * SAMPLE_FREQ)
SPI_WINDOW_N = 12
PACKET_SEPARATION = 120e-3
PLL_PHASE_GAIN = 0.5
PLL_CENTER_GAIN = 0.05

PIN_CLK = 1
PIN_DATA = 0

ctl = gp.GpioAsyncController()
ctl.configure('ftdi://ftdi:232h:1/1', frequency = SAMPLE_FREQ)

'''
Plan of attack:
1. aquire lock: read chunks of data until we see a packet in there
   a. spawn a delay thread that will trigger another read after PACKET_SEPARATION
2. decode the packet
3. once the delay happens read data again and then adjust the PACKET_SEPARATION 
   based on where the data is in the read signal (form the feedback of the PLL)
'''

def parse_data(bytes):
    arr = np.frombuffer(bytes, dtype=np.uint8)
    clk = np.bitwise_and(arr, 1<<PIN_CLK) != 0
    data = np.bitwise_and(arr, 1<<PIN_DATA) != 0
    return clk, data

def decode_spi(clk, data):
    #this is spi - sample the data on clock edges
    #there is an inverter in there so invert the data
    data = ~data
    clk = ~clk

    #grab the edges, data is clocked in on the rising edge
    edges = np.diff(clk.astype(np.int8)).astype(np.int8)

    if np.sum(np.abs(edges)) == 0:
        print("no edges found")
        return
    
    edges_idx = np.argwhere(edges == 1).flatten()
    
    #the first edge is the start of the packet, ignore it
    edges_idx = edges_idx[1:]
    
    #sample the data on the rising edge
    packet = data[edges_idx]


    powers = 1 << np.arange(16, dtype=np.uint16)[::]
    value = np.sum(powers * packet[:16])

    imperial = packet[-1]
    negative = packet[-4]
    sign = (1 - 2*negative)
    if imperial:
        return sign * value /10000
    else:
        return sign * value /500

def check_data(clk):
    #make sure that all the clk values are in the array
    edges = np.abs(np.diff(clk)).astype(np.uint8)
    assert edges.dtype == np.uint8
    if np.all(edges == 0):
        return False
    print("saw something!")
    #check that they are at least 1 window from the edges
    if np.any(edges[0:SPI_WINDOW] == 1):
        return False
    if np.any(edges[-SPI_WINDOW:] == 1):
        return False
    return True

def adjust_delay(clk):
    edges = np.abs(np.diff(clk)).astype(np.uint8)
    assert edges.dtype == np.uint8
    #find the first edge
    first_edge = np.argmax(edges)
    if first_edge == 0:
        return 0
    error = (first_edge - clk.size/2) / SAMPLE_FREQ
    return error

for _ in range(10000):
    data = ctl.read(int(SPI_WINDOW*SPI_WINDOW_N*10))
    clk, data = parse_data(data)
    if check_data(clk):
        print('detected signal')
        break
    else:
        print('no signal detected')
        raise Exception('no signal detected')
    

time.sleep(PACKET_SEPARATION - (SPI_WINDOW*SPI_WINDOW_N) / SAMPLE_FREQ)

class pll_reader:
    phase_error = 0
    centering_error = 0
    delays = np.zeros(2)

    def pll_read(self):
        threading.Timer(PACKET_SEPARATION - self.phase_error - self.centering_error, self.pll_read).start()
        data = ctl.read(SPI_WINDOW*SPI_WINDOW_N)
        clk, data = parse_data(data)
        if self.pll_adjust(clk):
            
            values = decode_spi(clk, data)
            print(values)
        
        
    def pll_adjust(self, clk):
        delay_error = adjust_delay(clk)
        print(delay_error)
        self.delays[1] = delay_error
        self.delays = np.unwrap(self.delays, period=SPI_WINDOW*SPI_WINDOW_N/SAMPLE_FREQ)

        delta_delay = self.delays[0] -self.delays[1]
        self.delays[0] = self.delays[1]
        
        # value = decode_packet(data)
        self.phase_error += PLL_PHASE_GAIN*delta_delay

        #push the signal to the center and correct the phase error since we are introducing some
        self.centering_error = -PLL_CENTER_GAIN*delay_error
        self.delays[0] -= PLL_CENTER_GAIN*delay_error

        n_edges = np.sum(np.abs(np.diff(clk)))
        if n_edges != 48:
            print("warning lost lock")
            return False
        return True

        # print(f"pll sees {n_edges} edges, error is {self.error:.3f} and phase error is {delta_delay:.3f}")


    
pll = pll_reader()
pll.pll_read()




