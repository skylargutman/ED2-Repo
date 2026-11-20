import os
import sys

# Tests import the picontrol modules (pilink, uart_link_check, dac_daemon) directly
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
