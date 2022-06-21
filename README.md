# Path loss Simulator v1.1  2022-06-13

Path Loss Simulator integrating ns-3 and SUMO is a testbed used in 5G NR V2X research. The network simulator ns–3 is the de-facto standard for academic and industry studies in the areas of networking protocols and communication technologies. Also SUMO is an open source, portable, microscopic and continuous multi-modal traffic simulation package designed to handle large networks.

# ns-3 Installation

1. Install all dependencies required by ns-3. 
see https://www.nsnam.org/wiki/Installation


  minimal requirements for C++:
  ```
  $apt-get install gcc g++ python
  ```
2. Clone git repository
  ```
  $git clone https://github.com/sangmosung/path-loss_simulator.git
  ```
3. Configure and build ns-3 project:
  ```
  $cd path-loss_simulator
  $./waf configure --disable-python --enable-examples && ./waf build
  ```
# TBU

# Contact
Sangmo Sung, UNLab, smsung@hanyang.ac.kr
