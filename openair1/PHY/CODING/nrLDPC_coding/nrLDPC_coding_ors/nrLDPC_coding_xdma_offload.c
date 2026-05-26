/*
 * Copyright (c) 2016-present,  Xilinx, Inc.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license
 * the terms of the BSD Licence are reported below:
 *
 * BSD License
 * 
 * For Xilinx DMA IP software
 * 
 * Copyright (c) 2016-present, Xilinx, Inc. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 *  * Neither the name Xilinx nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define _DEFAULT_SOURCE
#define _XOPEN_SOURCE 500

#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <byteswap.h>
#include <signal.h>
#include <ctype.h>
#include <termios.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include "xdma_diag.h"
#include "nrLDPC_coding_xdma_offload.h"

#include "common/utils/assertions.h"
#include "common/utils/nr/nr_common.h"

typedef unsigned long long U64;
void* map_base;
int fd = -1;
int fd_enc_write = -1, fd_enc_read = -1;
char *dev_enc_write, *dev_enc_read;
int fd_dec_write = -1, fd_dec_read = -1;
char *dev_dec_write, *dev_dec_read;
char allocated_write[24 * 1024] __attribute__((aligned(4096)));
char allocated_read[24 * 1024 * 3] __attribute__((aligned(4096)));
double cpu_freq_GHz;

static pthread_mutex_t hw_rw_lock;

// dma_from_device.c

// [Start] #include "dma_utils.c" ===================================

/*
 * man 2 write:
 * On Linux, write() (and similar system calls) will transfer at most
 * 	0x7ffff000 (2,147,479,552) bytes, returning the number of bytes
 *	actually transferred.  (This is true on both 32-bit and 64-bit
 *	systems.)
 */
#ifdef DEBUG_HW_HDR
  #define PRINT_ORS_TX_HEADER(tx_hdr) do { \
    printf("Tx hdr{max_schedule=%u, mb=%u, id=%u, max_iter=%u, toked_iter=%u, term_no_change=%u, term_pass=%u, include_parity=%u, hard_op_o=%u, sc_idx=%u, bg=%u, z_set=%u, z_j=%u, magic=0x%08x, payload_len=%u}\n", \
    (tx_hdr).max_schedule, \
    (tx_hdr).mb, \
    (tx_hdr).id, \
    (tx_hdr).max_iter, \
    (tx_hdr).toked_iter, \
    (tx_hdr).term_on_no_change, \
    (tx_hdr).term_on_pass, \
    (tx_hdr).include_parity_op, \
    (tx_hdr).hard_op_o, \
    (tx_hdr).sc_idx, \
    (tx_hdr).bg, \
    (tx_hdr).z_set, \
    (tx_hdr).z_j, \
    (tx_hdr).magic_field, \
    (tx_hdr).payload_len \
  ); \
    }while(0)
  #define PRINT_ORS_RX_HEADER(rx_hdr) do { \
    printf("Rx hdr{mb=%u, id=%u, toked_iter=%u, term_no_change=%u, term_pass=%u, parity_check_pass=%u, hard_op_o=%u, bg=%u, z_set=%u, z_j=%u, magic=0x%08x, payload_len=%u}\n", \
        (rx_hdr).mb, \
        (rx_hdr).id, \
        (rx_hdr).toked_iter, \
        (rx_hdr).term_on_no_change, \
        (rx_hdr).term_pass, \
        (rx_hdr).parity_check_pass, \
        (rx_hdr).hard_op_o, \
        (rx_hdr).bg, \
        (rx_hdr).z_set, \
        (rx_hdr).z_j, \
        (rx_hdr).magic_field, \
        (rx_hdr).payload_len); \
    }while(0)

#else
  #define PRINT_ORS_RX_HEADER(rx_hdr) ((void)(rx_hdr))
  #define PRINT_ORS_TX_HEADER(tx_hdr) ((void)(tx_hdr))
#endif
 typedef struct ors_tx_header_s {
  uint8_t max_schedule;
  uint8_t mb;
  uint8_t id;
  uint8_t max_iter;
  uint8_t toked_iter;
  uint8_t term_on_no_change;
  uint8_t term_on_pass;
  uint8_t include_parity_op;
  uint8_t hard_op_o;
  uint8_t sc_idx;
  uint8_t bg;
  uint8_t z_set;
  uint8_t z_j;
  uint32_t magic_field;
  uint32_t payload_len; //< numb of 16 bytes chunks
 } ors_tx_header_t;

 typedef struct ors_rx_header_s {
  uint8_t mb;
  uint8_t id;
  uint8_t toked_iter;
  uint8_t term_on_no_change;
  uint8_t term_pass;
  uint8_t parity_check_pass;
  uint8_t hard_op_o;
  uint8_t bg;
  uint8_t z_set;
  uint8_t z_j;
  uint32_t magic_field;
  uint32_t payload_len; //< numb of 16 bytes chunks
 } ors_rx_header_t;
#define ORS_MAGIC (0x7E7EU)
#define RW_MAX_SIZE 0x7ffff000

int verbose = 0;

void write_header_to_buffer(const ors_tx_header_t* hs, void* buffer, const size_t CB_num, const uint32_t* offsets)
{
  uint64_t tmp[2] = {0};
  //TODO optimize
  for(size_t r = 0; r < CB_num; ++r)
  {
    tmp[0] =
       (((uint64_t)hs[r].max_schedule & 0xF)  << 38) |
       (((uint64_t)hs[r].mb              & 0x3F) << 32) |
       (((uint64_t)hs[r].id           & 0xFF) << 24) |
       (((uint64_t)hs[r].max_iter     & 0x3F) << 18) |
       (((uint64_t)hs[r].term_on_no_change & 0x1) << 17) |
       (((uint64_t)hs[r].term_on_pass      & 0x1) << 16) |
       (((uint64_t)hs[r].include_parity_op & 0x1) << 15) |
       (((uint64_t)hs[r].hard_op_o         & 0x1) << 14) |
       (((uint64_t)hs[r].sc_idx       & 0xF)  << 9)  |
       (((uint64_t)hs[r].bg           & 0x7)  << 6)  |
       (((uint64_t)hs[r].z_set        & 0x7)  << 3)  |
       (((uint64_t)hs[r].z_j          & 0x7));
   
   tmp[1] =
       (((uint64_t)hs[r].magic_field & 0xFFFF) << 48) |
       (((uint64_t)hs[r].payload_len & 0xFFFF) << 32);
    
    memcpy(buffer + offsets[r], tmp, sizeof(tmp));
    memset(tmp, 0, sizeof(tmp));
  }
}

void read_headers_from_buffer(const void* buffer, ors_rx_header_t* headers, const size_t CB_num, const size_t offset)
{
  for(size_t r = 0; r < CB_num; ++r)
  {
    const size_t local_offset = offset * r;
    const uint64_t *w0 = (const uint64_t*)(buffer + local_offset);
    const uint64_t *w1 = (const uint64_t*)(buffer + local_offset + sizeof(uint64_t));
    ors_rx_header_t hd = {};
  
    hd.mb                 = (*w0 >> 32) & 0x3F;
    hd.id                 = (*w0 >> 24) & 0xFF;
    hd.toked_iter         = (*w0 >> 18) & 0x3F;
    hd.term_on_no_change  = (*w0 >> 17) & 0x1;
    hd.term_pass          = (*w0 >> 16) & 0x1;
    hd.parity_check_pass  = (*w0 >> 15) & 0x1;
    hd.hard_op_o          = (*w0 >> 14) & 0x1;
    hd.bg                 = (*w0 >> 6)  & 0x7;
    hd.z_set              = (*w0 >> 3)  & 0x7;
    hd.z_j                =  *w0        & 0x7;
  
    hd.magic_field        = (*w1 >> 48) & 0xFFFF;
    hd.payload_len        = (*w1 >> 32) & 0xFFFF;
    headers[r] = hd;
  }
}

uint64_t getopt_integer(char* optarg)
{
  int rc;
  uint64_t value;

  rc = sscanf(optarg, "0x%lx", &value);
  if (rc <= 0)
    rc = sscanf(optarg, "%lu", &value);

  return value;
}

ssize_t read_to_buffer(char* fname, int fd, char* buffer, uint64_t size, uint64_t base)
{
  ssize_t rc;
  uint64_t count = 0;
  char* buf = buffer;
  off_t offset = base;

  while (count < size) {
    uint64_t bytes = size - count;

    if (bytes > RW_MAX_SIZE)
      bytes = RW_MAX_SIZE;

    if (offset) {
      rc = lseek(fd, offset, SEEK_SET);
      if (rc != offset) {
        fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n", fname, rc, offset);
        perror("seek file");
        return -EIO;
      }
    }
    /* read data from file into memory buffer */
    rc = read(fd, buf, bytes);

    if (rc != bytes) {
      fprintf(stderr, "%s, R off 0x%lx, 0x%lx != 0x%lx.\n", fname, count, rc, bytes);
      perror("read file");
      return -EIO;
    }

    count += bytes;
    buf += bytes;
    offset += bytes;
  }

  if (count != size) {
    fprintf(stderr, "%s, R failed 0x%lx != 0x%lx.\n", fname, count, size);
    return -EIO;
  }
  return count;
}

ssize_t write_from_buffer(char* fname, int fd, char* buffer, uint64_t size, uint64_t base)
{
  ssize_t rc;
  uint64_t count = 0;
  char* buf = buffer;
  off_t offset = base;

  while (count < size) {
    uint64_t bytes = size - count;

    if (bytes > RW_MAX_SIZE)
      bytes = RW_MAX_SIZE;

    if (offset) {
      rc = lseek(fd, offset, SEEK_SET);
      if (rc != offset) {
        fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n", fname, rc, offset);
        perror("seek file");
        return -EIO;
      }
    }

    /* write data to file from memory buffer */
    rc = write(fd, buf, bytes);
    if (rc != bytes) {
      fprintf(stderr, "%s, W off 0x%lx, 0x%lx != 0x%lx.\n", fname, offset, rc, bytes);
      perror("write file");
      return -EIO;
    }

    count += bytes;
    buf += bytes;
    offset += bytes;
  }

  if (count != size) {
    fprintf(stderr, "%s, R failed 0x%lx != 0x%lx.\n", fname, count, size);
    return -EIO;
  }
  return count;
}

// [End] #include "dma_utils.c" ===================================

int test_dma_enc_read(char* EncOut, EncIPConf Confparam)
{
  ssize_t rc;

  void* virt_addr;

  uint64_t size;
  uint32_t writeval;

  uint32_t Z_val;

  uint16_t max_schedule, mb, id, bg, z_j, kb, z_a;
  uint16_t z_set;
  uint32_t ctrl_data;
  uint32_t CB_num = CB_PROCESS_NUMBER;

  // this values should be given by Shane
  max_schedule = 0;
  mb = Confparam.mb;
  id = CB_num;
  bg = Confparam.BGSel - 1;
  z_set = Confparam.z_set - 1;
  z_j = Confparam.z_j;

  if (z_set == 0)
    z_a = 2;
  else if (z_set == 1)
    z_a = 3;
  else if (z_set == 2)
    z_a = 5;
  else if (z_set == 3)
    z_a = 7;
  else if (z_set == 4)
    z_a = 9;
  else if (z_set == 5)
    z_a = 11;
  else if (z_set == 6)
    z_a = 13;
  else
    z_a = 15;

  if (bg == 0)
    kb = 22;
  else if (bg == 1)
    kb = 10;
  else if (bg == 2)
    kb = 9;
  else if (bg == 3)
    kb = 8;
  else
    kb = 6;
  mb = Confparam.kb_1 + kb;
  Z_val = (unsigned int)(z_a << z_j);
  ctrl_data = (max_schedule << 30) | ((mb - kb) << 24) | (id << 19) | (bg << 6) | (z_set << 3) | z_j;
  uint32_t OutDataNUM = Z_val * mb;
  uint32_t Out_dwNumItems_p128;
  uint32_t Out_dwNumItems;

  if ((OutDataNUM & 0x7F) == 0)
    Out_dwNumItems_p128 = OutDataNUM >> 5;
  else
    Out_dwNumItems_p128 = ((OutDataNUM >> 7) + 1) << 2;
  Out_dwNumItems = ((Out_dwNumItems_p128 << 2) * CB_num);
  size = Out_dwNumItems;
  writeval = ctrl_data;

  /* calculate the virtual address to be accessed */
  virt_addr = map_base + OFFSET_ENC_OUT;

  /* swap 32-bit endianess if host is not little-endian */
  writeval = htoll(writeval);
  *((uint32_t*)virt_addr) = writeval;
  if (fd_enc_read < 0) {
    fprintf(stderr, "unable to open device %s, %d.\n", dev_enc_read, fd_enc_read);
    perror("open device");
    return -EINVAL;
  }

  /* lseek & read data from AXI MM into buffer using SGDMA */
  rc = read_to_buffer(dev_enc_read, fd_enc_read, EncOut, size, 0);
  if (rc < 0)
    goto out;

  rc = 0;

out:

  return rc;
}

int test_dma_enc_write(char* data, EncIPConf Confparam)
{
  ssize_t rc;
  void* virt_addr;

  uint64_t size;
  uint32_t writeval;

  uint32_t Z_val;
  uint16_t max_schedule, mb, id, bg, z_j, kb, z_a;
  uint16_t z_set;
  uint32_t ctrl_data;
  uint32_t CB_num = CB_PROCESS_NUMBER;

  // this values should be given by Shane
  max_schedule = 0;

  mb = Confparam.mb;
  id = CB_num;
  bg = Confparam.BGSel - 1;
  z_set = Confparam.z_set - 1;
  z_j = Confparam.z_j;

  if (z_set == 0)
    z_a = 2;
  else if (z_set == 1)
    z_a = 3;
  else if (z_set == 2)
    z_a = 5;
  else if (z_set == 3)
    z_a = 7;
  else if (z_set == 4)
    z_a = 9;
  else if (z_set == 5)
    z_a = 11;
  else if (z_set == 6)
    z_a = 13;
  else
    z_a = 15;

  if (bg == 0)
    kb = 22;
  else if (bg == 1)
    kb = 10;
  else if (bg == 2)
    kb = 9;
  else if (bg == 3)
    kb = 8;
  else
    kb = 6;
  mb = Confparam.kb_1 + kb;
  Z_val = (unsigned int)(z_a << z_j);
  ctrl_data = (max_schedule << 30) | ((mb - kb) << 24) | (id << 19) | (bg << 6) | (z_set << 3) | z_j;
  uint32_t InDataNUM = Z_val * kb;
  uint32_t In_dwNumItems_p128;
  uint32_t In_dwNumItems;

  if ((InDataNUM & 0x7F) == 0)
    In_dwNumItems_p128 = InDataNUM >> 5;
  else
    In_dwNumItems_p128 = ((InDataNUM >> 7) + 1) << 2;

  In_dwNumItems = ((In_dwNumItems_p128 << 2) * CB_num);
  size = In_dwNumItems;
  writeval = ctrl_data;

  /* calculate the virtual address to be accessed */
  virt_addr = map_base + OFFSET_ENC_IN;

  /* swap 32-bit endianess if host is not little-endian */
  writeval = htoll(writeval);
  *((uint32_t*)virt_addr) = writeval;
  if (fd_enc_write < 0) {
    fprintf(stderr, "unable to open device %s, %d.\n", dev_enc_write, fd_enc_write);
    perror("open device");
    return -EINVAL;
  }

  rc = write_from_buffer(dev_enc_write, fd_enc_write, data, size, 0);
  if (rc < 0)
    goto out;
  rc = 0;

out:

  return rc;
}

// int test_dma_dec_read(unsigned int *DecOut, DecIPConf Confparam)
int test_dma_dec_read(char* DecOut, DecIPConf Confparam)
{
  ssize_t rc;

  uint64_t size;

  uint32_t Z_val;

  uint16_t bg, z_j, kb, z_a;
  uint16_t z_set;
  uint32_t CB_num = Confparam.CB_num;

  // this values should be given by Shane
  bg = Confparam.BGSel - 1;
  z_set = Confparam.z_set - 1;
  z_j = Confparam.z_j;

  if (z_set == 0)
    z_a = 2;
  else if (z_set == 1)
    z_a = 3;
  else if (z_set == 2)
    z_a = 5;
  else if (z_set == 3)
    z_a = 7;
  else if (z_set == 4)
    z_a = 9;
  else if (z_set == 5)
    z_a = 11;
  else if (z_set == 6)
    z_a = 13;
  else
    z_a = 15;

  if (bg == 0)
    kb = 22;
  else if (bg == 1)
    kb = 10;
  else if (bg == 2)
    kb = 9;
  else if (bg == 3)
    kb = 8;
  else
    kb = 6;

  Z_val = (unsigned int)(z_a << z_j);

  const uint32_t OutDataNUM = Z_val * kb + HEADER_SIZE * 8;
  const uint32_t Out_dwNumItems_p128 = CEIL_UP(OutDataNUM, 128);
  //bits to bytes
  const uint32_t Out_dwNumItems = Out_dwNumItems_p128 / 8;

  size = Out_dwNumItems * CB_num; 
  
  if (fd_dec_read < 0) {
    fprintf(stderr, "unable to open device %s, %d.\n", dev_dec_read, fd_dec_read);
    perror("open device");
    return -EINVAL;
  }

  /* read data from AXI ST into buffer using SGDMA */
  rc = read_to_buffer(dev_dec_read, fd_dec_read, DecOut, size, 0);
  ors_rx_header_t headers[MAX_CB];
  read_headers_from_buffer(DecOut, &headers[0], CB_num, Out_dwNumItems);
  PRINT_ORS_RX_HEADER(headers[0]);
  size_t mx_iter = 0;
  for(size_t r = 0; r < CB_num; ++r)
  {
    const uint16_t previous_set_id = max((CB_num - 1) - r, 255); 
    if(headers[r].id != previous_set_id)
    {
      printf("Header ID mismatch. ID should be %u got %u.\n", previous_set_id, headers[r].id);
      rc = -(EINVAL);
      break;
    }
    mx_iter = mx_iter > headers[r].toked_iter ? mx_iter : headers[r].toked_iter;

  }
  if (rc < 0)
    goto out;

  rc = mx_iter;
out:

  return rc;
}

// int test_dma_dec_write(unsigned int *data, DecIPConf Confparam)
int test_dma_dec_write(char* data, DecIPConf Confparam)
{
  ssize_t rc;

  uint64_t size = 0;

  uint32_t Z_val;
  uint16_t max_schedule, id, bg, z_j, kb, z_a, max_iter, sc_idx;
  uint16_t z_set;
  uint32_t CB_num = Confparam.CB_num; // CB_PROCESS_NUMBER_Dec;//

  // this values should be given by Shane
  max_schedule = 0;
  id = CB_num - 1;
  bg = Confparam.BGSel - 1;
  z_set = Confparam.z_set - 1;
  z_j = Confparam.z_j;
  
  max_iter = Confparam.max_iter;
  sc_idx = Confparam.SetIdx;

  if (z_set == 0)
    z_a = 2;
  else if (z_set == 1)
    z_a = 3;
  else if (z_set == 2)
    z_a = 5;
  else if (z_set == 3)
    z_a = 7;
  else if (z_set == 4)
    z_a = 9;
  else if (z_set == 5)
    z_a = 11;
  else if (z_set == 6)
    z_a = 13;
  else
    z_a = 15;

  if (bg == 0)
  {
    kb = 22;
  }
  else if (bg == 1)
  {
    kb = 10;
  }
  else if (bg == 2)
  {
    kb = 9;
  }
  else if (bg == 3)
  {
    kb = 8;
  }
  else
  {
    kb = 6;
  }

  Z_val = (unsigned int)(z_a << z_j);
  ors_tx_header_t headers[MAX_CB] = {}; 
  uint32_t offsets[MAX_CB] = {};
  for(uint32_t r = 0; r < CB_num; ++r)
  {
    const uint8_t mb = Confparam.numb_of_parity_bits_per_cb[r] / Z_val;
    //kb (number of informations bits)
    size_t local_size = HEADER_SIZE + kb * Z_val + Confparam.numb_of_parity_bits_per_cb[r];
    //ceil up to a multiple of 16B
    local_size = CEIL_UP_16B(local_size);
    offsets[r] = size;  
    size += local_size;
    const size_t numb_of_16B_units = (local_size - HEADER_SIZE) / 16; //< header isn't part of data
    headers[r].max_schedule = max_schedule;
    headers[r].mb = mb;
    headers[r].id = max(255, id);
    headers[r].max_iter = max_iter;
    headers[r].term_on_no_change = 1;
    headers[r].term_on_pass = 1;
    headers[r].include_parity_op = 0;
    headers[r].hard_op_o = 1;
    headers[r].sc_idx = sc_idx;
    headers[r].bg = bg;
    headers[r].z_set = z_set;
    headers[r].z_j = z_j;
    headers[r].magic_field = ORS_MAGIC;
    headers[r].payload_len = numb_of_16B_units; //< payload size in 16 Byte units
    PRINT_ORS_TX_HEADER(headers[r]);
    id--;
  }
  //from bytes to 16 byte units
  //insert header infront of data
  write_header_to_buffer(headers, data, CB_num, offsets);

  if (fd_dec_write < 0) {
    fprintf(stderr, "unable to open device %s, %d.\n", dev_dec_write, fd_dec_write);
    perror("open device");
    return -EINVAL;
  }

  rc = write_from_buffer(dev_dec_write, fd_dec_write, data, size, 0);
  if (rc < 0)
    goto out;

  rc = 0;

out:

  return rc;
}

void init_hw_timer()
{
  volatile uint32_t* base_hw_addr =  (volatile uint32_t*)(((uint8_t*)map_base) + OFFSET_AXI_TIMER);
  for(uint32_t i = 0; i < 2; ++i)
  {
    base_hw_addr[i * 4 + 1] = 0;
    base_hw_addr[i * 4] |= 1U;
    base_hw_addr[i * 4] &= ~(1U << 1);
    base_hw_addr[i * 4] |= 1U << 3;
    base_hw_addr[i * 4] &= ~(1U << 4);
    base_hw_addr[i * 4] |= (1U << 5);
    base_hw_addr[i * 4] &= ~(1U << 5);
    if(i == 1)
    {
      base_hw_addr[i * 4] |= (1 << 4);
    }
  }
}

void start_hw_timer()
{
  volatile uint32_t* base_hw_addr =  (volatile uint32_t*)(((uint8_t*)map_base) + OFFSET_AXI_TIMER);
  for(uint32_t i = 0; i < 2; ++i)
  {
    base_hw_addr[i * 4] &= ~(1U << 7);
    base_hw_addr[i * 4 + 1] = 0;
    base_hw_addr[i * 4] |= (1U << 5);
    base_hw_addr[i * 4] &= ~(1U << 5);
  }
  //enable all timers
  base_hw_addr[0] |= (1 << 10);
}

uint32_t get_hw_valid_ticks()
{
  volatile uint32_t* base_hw_addr =  (volatile uint32_t*)(((uint8_t*)map_base) + OFFSET_AXI_TIMER);
  return base_hw_addr[1];
}

uint32_t get_hw_dec_latency_ticks()
{
  volatile uint32_t* base_hw_addr =  (volatile uint32_t*)(((uint8_t*)map_base) + OFFSET_AXI_TIMER);
  const uint32_t end = base_hw_addr[5];
  const uint32_t start = get_hw_valid_ticks();
  return end - start;
}

int32_t test_dma_init(devices_t devices)
{
  pthread_mutex_init(&hw_rw_lock, NULL);
  int32_t ret = 0;
  
  //device files already opened
  if(fd_dec_write > 0 && fd_dec_read > 0 && fd > 0)
  {
    return ret;
  }

  dev_dec_write = devices.dec_write_device;
  dev_dec_read = devices.dec_read_device;

  fd_dec_write = open(dev_dec_write, O_WRONLY);
  if(fd_dec_write < 0)
  {
    ret = fd_dec_write;
    printf("Failed to open %s!", dev_dec_write);
    goto test_dma_out;
  }
  fd_dec_read = open(dev_dec_read, O_RDONLY);
  if(fd_dec_read < 0)
  {
    ret = fd_dec_read;
    printf("Failed to open %s!", dev_dec_read);
    close(fd_dec_write);
    goto test_dma_out;
  }

  fd = open(devices.user_device, O_RDWR | O_SYNC);
  if(fd < 0)
  {
    printf("Failed to open %s!", devices.user_device);
    close(fd_dec_write);
    close(fd_dec_read);
    ret = fd;
    goto test_dma_out;
  }

  map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  AssertFatal(map_base != (void*)-1, "MEMORY MAP AT ADDRESS %p FAILED\n", map_base);

  *(volatile uint32_t*)(map_base + OFFSET_RESET) |= (1 << 8);
  usleep(10);
  *(volatile uint32_t*)(map_base + OFFSET_RESET) &= ~(1 << 8);

  cpu_freq_GHz = get_cpu_freq_GHz();

  init_hw_timer();

  fflush(stdout);
  test_dma_out:
  return ret;

}

void dma_close()
{
  pthread_mutex_destroy(&hw_rw_lock);
  *(volatile uint32_t*)(map_base + OFFSET_RESET) |= (1 << 8);
  *(volatile uint32_t*)(map_base + OFFSET_RESET) &= ~(1 << 8);
  munmap(map_base, MAP_SIZE);
  if(fd_dec_write > 0)
    close(fd_dec_write);
  if(fd_dec_read > 0)
    close(fd_dec_read);
  if(fd > 0)
    close(fd);
  fd_dec_write = -1;
  fd_dec_read = -1;
  fd = -1;
}

void dma_reset(devices_t devices)
{
  char* device2 = devices.user_device;

  void* virt_addr;
  virt_addr = map_base + PCIE_OFF;
  *((uint32_t*)virt_addr) = 1;

  AssertFatal(munmap(map_base, MAP_SIZE) != -1, "munmap failure");
  close(fd_enc_write);
  close(fd_enc_read);
  close(fd_dec_write);
  close(fd_dec_read);
  close(fd);

  AssertFatal((fd = open(device2, O_RDWR | O_SYNC)) != -1, "CHARACTER DEVICE %s OPEN FAILURE\n", device2);
  fflush(stdout);

  /* map one page */
  map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  AssertFatal(map_base != (void*)-1, "MEMORY MAP AT ADDRESS %p FAILED\n", map_base);

  virt_addr = map_base + PCIE_OFF;
  *((uint32_t*)virt_addr) = 1;

  virt_addr = map_base + OFFSET_RESET;
  *((uint32_t*)virt_addr) = 1;

  dev_enc_write = devices.enc_write_device;
  dev_enc_read = devices.enc_read_device;
  dev_dec_write = devices.dec_write_device;
  dev_dec_read = devices.dec_read_device;

  fd_enc_write = open(dev_enc_write, O_RDWR);
  fd_enc_read = open(dev_enc_read, O_RDWR);
  fd_dec_write = open(dev_dec_write, O_RDWR);
  fd_dec_read = open(dev_dec_read, O_RDWR);

  fflush(stdout);
}

void test_dma_shutdown()
{

  void* virt_addr;
  virt_addr = map_base + PCIE_OFF;
  *((uint32_t*)virt_addr) = 1;

  AssertFatal(munmap(map_base, MAP_SIZE) != -1, "munmap failure");
  close(fd_enc_write);
  close(fd_enc_read);
  close(fd_dec_write);
  close(fd_dec_read);
  close(fd);
}

static void conv_hwtime2cputime(const uint32_t hw_ticks, time_stats_t* time)
{
  const double hw_freq_GHz = 0.25;
  const double factor = cpu_freq_GHz/hw_freq_GHz;
  time->trials++;
  const oai_cputime_t cpu_ticks = (oai_cputime_t)(((double)hw_ticks)*factor);
  time->diff += cpu_ticks;
  time->diff_square += ((double)cpu_ticks) * ((double)cpu_ticks);
  time->max = time->max > cpu_ticks ?time->max : cpu_ticks;
  time->p_time = cpu_ticks;
  time->meas_flag = 0;
}

// reg_rx.c
int nrLDPC_decoder_FPGA_PYM(uint8_t* buf_in, uint8_t* buf_out, DecIFConf dec_conf)
{
  int Zc;
  int nRows;
  int baseGraph;
  int CB_num;

  DecIPConf Confparam = {
    .max_iter = dec_conf.max_iter, 
    .SetIdx = dec_conf.SetIdx, 
    .numb_of_parity_bits_per_cb = dec_conf.numb_of_parity_bits_per_CB
  };
  int z_a, z_tmp;
  int z_j = 0;

  uint8_t i_LS;

  // LDPC input parameter
  Zc = dec_conf.Zc; // shifting size
  nRows = dec_conf.nRows; // number of Rows
  baseGraph = dec_conf.BG; // base graph
  CB_num = dec_conf.numCB; // 31 number of code block

  // calc xdma LDPC parameter
  // calc i_LS
  if ((Zc % 15) == 0)
    i_LS = 7;
  else if ((Zc % 13) == 0)
    i_LS = 6;
  else if ((Zc % 11) == 0)
    i_LS = 5;
  else if ((Zc % 9) == 0)
    i_LS = 4;
  else if ((Zc % 7) == 0)
    i_LS = 3;
  else if ((Zc % 5) == 0)
    i_LS = 2;
  else if ((Zc % 3) == 0)
    i_LS = 1;
  else
    i_LS = 0;

  // calc z_a
  if (i_LS == 0)
    z_a = 2;
  else
    z_a = i_LS * 2 + 1;

  // calc z_j
  z_tmp = Zc / z_a;
  while (z_tmp % 2 == 0) {
    z_j = z_j + 1;
    z_tmp = z_tmp / 2;
  }

  // calc CB_num and mb
  Confparam.CB_num = CB_num;
  Confparam.mb = nRows;

  // set BGSel, z_set, z_j
  Confparam.BGSel = baseGraph;
  Confparam.z_set = i_LS + 1;
  Confparam.z_j = z_j;
  pthread_mutex_lock(&hw_rw_lock);
  // LDPC accelerator start
  start_hw_timer();
  if(dec_conf.dec_write_time != NULL)
    start_meas(dec_conf.dec_write_time);
  // write into accelerator
  if (test_dma_dec_write((char *)buf_in, Confparam) != 0) {
    exit(1);
    printf("write exit!!\n");
  }
  if(dec_conf.dec_write_time != NULL)
    stop_meas(dec_conf.dec_write_time);
  // read output of accelerator
  if(dec_conf.dec_read_time != NULL)
    start_meas(dec_conf.dec_read_time);
  const int numb_of_iter_or_err = test_dma_dec_read((char *)buf_out, Confparam);
  if (numb_of_iter_or_err < 0) {
    exit(1);
    printf("read exit!!\n");
  }
  if(dec_conf.dec_read_time != NULL)
    stop_meas(dec_conf.dec_read_time);
  const uint32_t hw_ticks = get_hw_dec_latency_ticks();
  if(dec_conf.hw_dec_time != NULL)
    conv_hwtime2cputime(hw_ticks, dec_conf.hw_dec_time);
  if(dec_conf.h2c_latency != NULL)
  {
    const uint32_t time_to_valid_ticks = get_hw_valid_ticks(); 
    conv_hwtime2cputime(time_to_valid_ticks, dec_conf.h2c_latency);
  }
  pthread_mutex_unlock(&hw_rw_lock);


  return numb_of_iter_or_err;
}

