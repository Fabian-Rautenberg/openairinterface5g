# LDPC decoding

## Acronym List
K: Code Block size in Bits (max. value is 8448). Output of CB segmentation. It includes filler bits, CRC (if inserted), and bits from TB. If BG1 K=22*Z_c; If BG2 K=10*Z_c
K': Code block size in Bits without filler bits: K-F or B'/C
N: Number of total encoded bits for one CB. Value is for BG1-> 66*Z_c; BG2 -> 50*Z_c  
CB: Code Block
H: Parity Check Matrix
BG: Base Graph; BG1 -> 46x48; BG2 -> 42x52
TB: Transport Block
Z_c/Z: Lifting size
A: Transport Block size in Bits
C: Numb of segments/CB
L: Numb of parity bits 
B: Total TB size with CRC B = A + L
R: Code Rate R=#m/#c=#m/(#m+redundance bits)  
K_cb: Max. possible CB size in Bits
E: Number of Bits of one CB after Rate matching
F: number of fill bits. TODO: What are they related to? Are they needed to K_r match K 
iLS: Lifting size index
mb: Number of parity bits (This has to be N-K)

## Run lpdctest 
1. Go to the directory `<path_to_oai>/cmake_targets/`
2. Install dependencies: `sudo ./build_oai -I`. This has to be done just once.
3. Build `lpdctest`: `sudo ./build_oai -P -g Debug`. This will also build `libldpc` libraries. `-g Debug` for debug build, if not specified release mode will be used.
4. Go to the directory: `<path_to_oai>/cmake_targets/ran_build/build`
4. Execute `lpdctest`: `./ldpctest` SW version. `sudo ./ldpctest -v _ors` HW version. Use `./ldpctest -h` for all available commands.

## Furhter notes
The LDPC licenz is only valid for about 8 hours after that the card has to be reseted or reprogrammed. 
