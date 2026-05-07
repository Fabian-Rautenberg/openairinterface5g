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

## To be done:

- Test with different K' values. At least this values should be tested. Recommend intermediate values as well.
    - 192 (ldpctest checked with high SNR value)
    - 560 (ldpctest checked with high SNR value)
    - 600 (ldpctest checked with high SNR value)
    - 2200 (ldpctest checked with high SNR value)
    - 3840 (ldpctest checked with high SNR value)
    - 8448 (ldpctest checked with high SNR value)

## SW architecture Overview
Inside the directory nrLDPC_coding_segment/ is the slot decoding implementation in SW. The entry point is the function nrLDPC_coding_decoder, which gets multiple slots passed. It iterates over all slots and waits at the end for all started tasked (threads) to be completed. The function nrLDPC_prepare_TB_decoding is called for every slot. Inside this function it is iterrated over all CBs and the function pushes tasks on the pool, which can be handeld in parallel. The task to handle is nr_process_decode_segment. Here are things done like deinterleaving, derate matching and decoding. After the decoding the task is finished. The decoding call is done to LDPCdecoder (nrLDPC_decoder/ SW implementation (for one segment)).

Inside the directory nrLDPC_coding_xdma/ is the slot decoding for the XDMA HW-Acc offloading. It uses the slot decoding interface (nrLDPC_coding_decoder). It iterates over all slots. And pass one slot to decoder_xdma. There it iterarets over all code blocks and do deinterleaving and deratematching parallel. Tasked are again pushed to the threadpool. When all CBs are done deinterleaved and deratematched decode is called for all CBs. This is done in the function nrLDPC_decoder_FPGA_PYM. All CBs are transferred in one DMA write. The result is also read in just one DMA transfer. This makes effective use of PCIe throughput. It allows parallelization one the HW with multiple LDPC cores. HW is responsible of distributing the work among them. Also in the pure SW solution the main thread (work pusher) waits until all jobs (CBs) are done.

SW imp:
Job/CB
----------------
|deinterleaving|
|deratematching|
|decoding      |
----------------
wait for all slots

XDMA offloading:
Job/CB
----------------
|deinterleaving|
|deratematching|
|              |
----------------
wait for this job is done (one slot)
decoding for one slot (all CBs), when this is done no other slot is handeld (not 100% efficient/depends on the decoding time)

Start with offloading one CB (SW imp) then continue adapt the SW/HW interface to fulfill the XDMA offloading, because transferring multiple CBs in one DMA transfer is more efficient and allows parallel use of multiple decoder cores.
