# Path loss Simulator v1.1

Path Loss simulator integrating ns-3 and SUMO is a testbed used in 5G NR V2X research. The network simulator ns–3 is the de-facto standard for academic and industry studies in the areas of networking protocols and communication technologies. Also SUMO is an open source, portable, microscopic and continuous multi-modal traffic simulation package designed to handle large networks.

# ns-3 Installation

1. Install all dependencies required by ns-3. 
see https://www.nsnam.org/wiki/Installation
  minimal requirements for C++:
  ```
  $ apt-get install gcc g++ python
  ```
2. Install Git
  ```
  $ apt install git
  ```
3. Clone git repository
  ```
  $ git clone https://github.com/sangmosung/path-loss_simulator.git
  ```
4. Configure and build ns-3 project:
  ```
  $ cd path-loss_simulator
  $ ./waf configure --disable-python --enable-examples && ./waf build
  ```
# Contributors
TBU

# Contact
Sangmo Sung, UNLab, smsung@hanyang.ac.kr

# License
This software is licensed under the terms of the GNU GPLv2, as like as ns-3. See the LICENSE file for more details.

# Last update 
2022-06-13
