/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*! \file PHY/CODING/nrLDPC_coding/nrLDPC_coding_xdma/nrLDPC_coding_xdma.c
 * \brief Top-level routines for decoding LDPC (ULSCH) transport channels
 * decoding implemented using a FEC IP core on FPGA through XDMA driver
 */

// [from gNB coding]
#include <syscall.h>

#include <nr_rate_matching.h>
#include "PHY/CODING/coding_defs.h"
#include "PHY/CODING/coding_extern.h"
#include "PHY/CODING/nrLDPC_coding/nrLDPC_coding_ors/nrLDPC_coding_xdma_offload.h"
#include "PHY/CODING/nrLDPC_extern.h"
#include "common/utils/LOG/log.h"
#include "defs.h"
// #define DEBUG_ULSCH_DECODING
// #define gNB_DEBUG_TRACE

#define OAI_UL_LDPC_MAX_NUM_LLR (27000U) // 26112 // NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX = 68*384
#define MAX_CB_SIZE_IN_BYTE_UNITS (1100U) // 8488/8 -> 1056 
#define NUMB_OF_MAX_DEC_ITER (63U)
#define NUMB_OF_MIN_DEC_ITER (1U)
// #define DEBUG_CRC
#ifdef DEBUG_CRC
#define PRINT_CRC_CHECK(a) a
#else
#define PRINT_CRC_CHECK(a)
#endif


#include "nfapi/open-nFAPI/nfapi/public_inc/nfapi_interface.h"
#include "nfapi/open-nFAPI/nfapi/public_inc/nfapi_nr_interface.h"

#include "PHY/CODING/nrLDPC_coding/nrLDPC_coding_interface.h"

#include "xdma_diag.h"

// Global var to limit the rework of the dirty legacy code
int num_threads_prepare_max = 0;
char *user_device = NULL;
char *enc_read_device = NULL;
char *enc_write_device = NULL;
char *dec_read_device = NULL;
char *dec_write_device = NULL;

/*!
 * \typedef args_fpga_decode_prepare_t
 * \struct args_fpga_decode_prepare_s
 * \brief arguments structure for passing arguments to the nr_ulsch_FPGA_decoding_prepare_blocks function
 */
typedef struct args_fpga_decode_prepare_s {
  nrLDPC_TB_decoding_parameters_t *TB_params; /*!< transport blocks parameters */

  uint8_t *multi_indata; /*!< pointer to the head of the block destination array that is then passed to the FPGA decoding */
  int no_iteration_ldpc; /*!< pointer to the number of iteration set by this function */
  uint32_t r_first; /*!< index of the first block to be prepared within this function */
  uint32_t r_span; /*!< number of blocks to be prepared within this function */
  int r_offset; /*!< r index expressed in bits */
  int input_CBoffset; /*!< */
  int Kc; /*!< ratio between the number of columns in the parity check graph and the lifting size */
  int Kprime; /*!< size of payload and CRC bits in a code block */
  task_ans_t *ans; /*!< pointer to the answer that is used by thread pool to detect job completion */
} args_fpga_decode_prepare_t;

int32_t ors_nrLDPC_coding_init(void);
int32_t ors_nrLDPC_coding_shutdown(void);
int32_t ors_nrLDPC_coding_decoder(nrLDPC_slot_decoding_parameters_t *slot_params, int frame_rx, int slot_rx);
// int32_t nrLDPC_coding_encoder(void);
int decoder_xdma(nrLDPC_TB_decoding_parameters_t *TB_params, int frame_rx, int slot_rx, tpool_t *ldpc_threadPool);
void nr_ulsch_FPGA_decoding_prepare_blocks(void *args);

/**
 * To support segment decoding as well, the following function has to be implemented.
 */
int32_t LDPCinit(void);
int32_t LDPCshutdown(void);
int32_t LDPCdecoder(t_nrLDPC_dec_params *p_decParams, int8_t *p_llr, int8_t *p_out, t_nrLDPC_time_stats *time_stats, decode_abort_t *ab);
static uint8_t reverse_8bit(uint8_t byte);
int32_t LDPCinit(void)
{
  devices_t dev = {.dec_read_device = DEVICE_NAME_DEFAULT_DEC_READ,
                   .dec_write_device = DEVICE_NAME_DEFAULT_DEC_WRITE,
                  .user_device = DEVICE_NAME_DEFAULT_USER};
  int32_t ret = test_dma_init(dev);
  if(ret < 0)
  {
    printf("Unable to use Dec HW ACC!\n");
    exit(1);
  }
  return ret;
}

int32_t LDPCshutdown(void)
{
  dma_close();
  return 0;
}

// decoder interface
/**
   \brief LDPC decoder API type definition
   \param p_decParams LDPC decoder parameters
   \param p_llr Input LLRs
   \param p_llrOut Output vector
   \param time_stats time statistics
   \param ab structure shared between tasks to stop all the tasks if one fails
*/
int32_t LDPCdecoder(t_nrLDPC_dec_params *p_decParams, int8_t *p_llr, int8_t *p_out, t_nrLDPC_time_stats *time_stats, decode_abort_t *ab)
{
  DecIFConf dec_conf = {0};
  dec_conf.Zc = p_decParams->Z;
  dec_conf.BG = p_decParams->BG;
  //select correct BG 
  int Kb = 0;
  if(p_decParams->BG == 1)
  {
    Kb = 22;
    dec_conf.BG = 1;
  }
  else //second BG
  {
    #define USE_EXACT_BG (true)  
    #if USE_EXACT_BG 
    //The following has to be valid K_b * Z_c >= K'
    if(6 * p_decParams->Z >= p_decParams->Kprime)
    {
      Kb = 6;
      dec_conf.BG = 5;
    }
    else if(8 * p_decParams->Z >= p_decParams->Kprime)
    {
      Kb = 8;
      dec_conf.BG = 4;
    }
    else if(9 * p_decParams->Z >= p_decParams->Kprime)
    {
      Kb = 9;
      dec_conf.BG = 3;
    }
    else // 10 * Z_c >= K'
    {
      Kb = 10;
      dec_conf.BG = 2;
    }
    #else
    Kb = 10;
    dec_conf.BG = 2;
    #endif
    
  }
  dec_conf.max_iter = min(max(p_decParams->numMaxIter, NUMB_OF_MIN_DEC_ITER), NUMB_OF_MAX_DEC_ITER);
  dec_conf.numCB = 1; 
  // input soft bits length; not sure if calculation is correct
  dec_conf.numChannelLls = p_decParams->Kprime;
  // filler bits length
  dec_conf.numFillerBits = 0;
  dec_conf.max_schedule = 0;
  dec_conf.SetIdx = 11;
  //number of message words/number of rows in BG
  dec_conf.nRows = (p_decParams->BG == 1) ? 46 : 42;

  dec_conf.user_device = user_device;
  dec_conf.enc_read_device = enc_read_device;
  dec_conf.enc_write_device = enc_write_device;
  dec_conf.dec_read_device = dec_read_device;
  dec_conf.dec_write_device = dec_write_device;
  
  #define MAX_IN_DEC_ARRAY_SIZE (OAI_UL_LDPC_MAX_NUM_LLR + HEADER_SIZE)
  #define MAX_OUT_DEC_ARRAY_SIZE (MAX_CB_SIZE_IN_BYTE_UNITS + HEADER_SIZE)
  int8_t buffer_in[MAX_IN_DEC_ARRAY_SIZE];
  int8_t buffer_out[MAX_OUT_DEC_ARRAY_SIZE];

  const int N = p_decParams->BG == 2 ? 50 * p_decParams->Z : 66 * p_decParams->Z;
  const int K = p_decParams->BG == 2 ? 10 * p_decParams->Z : 22 * p_decParams->Z;
  const int punctured_bits = 2 * p_decParams->Z; 
  const int8_t max_level = 120;
  //copy all LLRs in internal buffer starting at HEADER_SIZE
  start_meas(&time_stats->total);
  //copy puncutred bits
  memcpy(&buffer_in[HEADER_SIZE], p_llr, punctured_bits * sizeof(*p_llr));
  //copy information bits
  for(int i = punctured_bits; i < Kb * p_decParams->Z; ++i)
  {
    if(p_llr[i] > max_level)
      buffer_in[HEADER_SIZE + i] = max_level;
    else if(p_llr[i] < -max_level)
      buffer_in[HEADER_SIZE + i] = -max_level;
    else
      buffer_in[HEADER_SIZE + i] = p_llr[i];
  }
  //copy parity bits
  for(int i = Kb * p_decParams->Z, j = K; j < N + punctured_bits; ++i, ++j)
  {
    if(p_llr[j] > max_level)
      buffer_in[HEADER_SIZE + i] = max_level;
    else if(p_llr[j] < -max_level)
      buffer_in[HEADER_SIZE + i] = -max_level;
    else
      buffer_in[HEADER_SIZE + i] = p_llr[j];
  }
  start_meas(&time_stats->llr2bit);
  int32_t niter = nrLDPC_decoder_FPGA_PYM((uint8_t*)&buffer_in[0], (uint8_t*)&buffer_out[0], dec_conf);
  stop_meas(&time_stats->llr2bit);
  int cK = Kb * p_decParams->Z; 
  if((cK % 8) != 0)
  {
    cK = (cK + 7) / 8; //ceil up
  }
  start_meas(&time_stats->llrRes2llrOut);
  //copy into out buffer and reverse bits
  for(int i = 0; i < cK; ++i)
  {
    p_out[i] = (int8_t)reverse_8bit((uint8_t)buffer_out[HEADER_SIZE + i]);
  }
  //set the remaining bits to zero
  const int rem = ((K - cK) + 7) / 8;
  memset(p_out + cK, 0, rem);
  stop_meas(&time_stats->llrRes2llrOut);
  stop_meas(&time_stats->total);
  if(p_decParams->check_crc != NULL)
  {
    if (!p_decParams->check_crc((uint8_t*)p_out, p_decParams->Kprime, p_decParams->crc_type)) 
    {
      LOG_D(PHY, "Segment CRC NOK!\n");
    }
  }
  return niter;
} 

static uint8_t reverse_8bit(uint8_t byte)
{
  byte = ((byte & 0xF0U) >> 4) | ((byte & 0x0FU) << 4);
  byte = ((byte & 0xCCU) >> 2) | ((byte & 0x33U) << 2);
  byte = ((byte & 0xAAU) >> 1) | ((byte & 0x55U) << 1);
  return byte;
}





int32_t ors_nrLDPC_coding_init(void)
{
  paramdef_t LoaderParams[] = {
      {"num_threads_prepare", NULL, 0, .iptr = &num_threads_prepare_max, .defintval = 0, TYPE_INT, 0, NULL},
      {"user_device", NULL, 0, .strptr = &user_device, .defstrval = DEVICE_NAME_DEFAULT_USER, TYPE_STRING, 0, NULL},
      {"enc_read_device", NULL, 0, .strptr = &enc_read_device, .defstrval = DEVICE_NAME_DEFAULT_ENC_READ, TYPE_STRING, 0, NULL},
      {"enc_write_device", NULL, 0, .strptr = &enc_write_device, .defstrval = DEVICE_NAME_DEFAULT_ENC_WRITE, TYPE_STRING, 0, NULL},
      {"dec_read_device", NULL, 0, .strptr = &dec_read_device, .defstrval = DEVICE_NAME_DEFAULT_DEC_READ, TYPE_STRING, 0, NULL},
      {"dec_write_device", NULL, 0, .strptr = &dec_write_device, .defstrval = DEVICE_NAME_DEFAULT_DEC_WRITE, TYPE_STRING, 0, NULL}};
  config_get(config_get_if(), LoaderParams, sizeofArray(LoaderParams), "nrLDPC_coding_xdma");
  AssertFatal(num_threads_prepare_max != 0, "nrLDPC_coding_xdma.num_threads_prepare was not provided");

  return 0;
}

int32_t ors_nrLDPC_coding_shutdown(void)
{
  return 0;
}

int32_t ors_nrLDPC_coding_decoder(nrLDPC_slot_decoding_parameters_t *slot_params, int frame_rx, int slot_rx)
{
  int nbDecode = 0;
  for (int ULSCH_id = 0; ULSCH_id < slot_params->nb_TBs; ULSCH_id++)
    nbDecode += decoder_xdma(&slot_params->TBs[ULSCH_id], frame_rx, slot_rx, slot_params->threadPool);
  return nbDecode;
}

/*
int32_t nrLDPC_coding_encoder(void)
{
  return 0;
}
*/

int decoder_xdma(nrLDPC_TB_decoding_parameters_t *TB_params, int frame_rx, int slot_rx, tpool_t *ldpc_threadPool)
{
  const uint32_t K = TB_params->K;
  //numb of columns in the BG (52 for BG2 and 68 for BG1) -> number of coded words
  const int Kc = TB_params->BG == 2 ? 52 : 68;
  int r_offset = 0, offset = 0;
  //number of true information bits; could also include CRC of CB
  int Kprime = K - TB_params->F;

  // FPGA parameter preprocessing
  static uint8_t multi_indata[OAI_UL_LDPC_MAX_NUM_LLR * 25 + HEADER_SIZE]; // FPGA input data
  static uint8_t multi_outdata[1100 * 25 + HEADER_SIZE]; // FPGA output data

  //maximum possible K_b value
  int bg_len = TB_params->BG == 1 ? 22 : 10;

  // Calc input CB offset
  int input_CBoffset = TB_params->Z * Kc * 8;
  if ((input_CBoffset & 0x7F) == 0)
    input_CBoffset = input_CBoffset / 8;
  else
    input_CBoffset = 16 * ((input_CBoffset / 128) + 1);

  DecIFConf dec_conf = {0};
  dec_conf.Zc = TB_params->Z;
  dec_conf.BG = TB_params->BG;
  dec_conf.max_iter = TB_params->max_ldpc_iterations;
  dec_conf.numCB = TB_params->C;
  // input soft bits length, Zc x 66 - length of filler bits
  dec_conf.numChannelLls = (Kprime - 2 * TB_params->Z) + (Kc * TB_params->Z - K);
  // filler bits length
  dec_conf.numFillerBits = TB_params->F;
  dec_conf.max_schedule = 0;
  dec_conf.SetIdx = 12;
  //number of message words/number of rows in BG
  dec_conf.nRows = (dec_conf.BG == 1) ? 46 : 42;

  dec_conf.user_device = user_device;
  dec_conf.enc_read_device = enc_read_device;
  dec_conf.enc_write_device = enc_write_device;
  dec_conf.dec_read_device = dec_read_device;
  dec_conf.dec_write_device = dec_write_device;
  //Z_c * (10 or 22) is K; Calculate the number of bytes in one CB
  int out_CBoffset = dec_conf.Zc * bg_len;
  if ((out_CBoffset & 0x7F) == 0)
    out_CBoffset = out_CBoffset / 8;
  else
    out_CBoffset = 16 * ((out_CBoffset / 128) + 1);

#ifdef LDPC_DATA
  printf("\n------------------------\n");
  printf("BG:\t\t%d\n", dec_conf.BG);
  printf("TB_params->C: %d\n", TB_params->C);
  printf("TB_params->K: %d\n", TB_params->K);
  printf("TB_params->Z: %d\n", TB_params->Z);
  printf("TB_params->F: %d\n", TB_params->F);
  printf("numChannelLls:\t %d = (%d - 2 * %d) + (%d * %d - %d)\n",
         dec_conf.numChannelLls,
         Kprime,
         TB_params->Z,
         Kc,
         TB_params->Z,
         K);
  printf("numFillerBits:\t %d\n", TB_params->F);
  printf("------------------------\n");
  // ===================================
  // debug mode
  // ===================================
  FILE *fptr_llr, *fptr_ldpc;
  fptr_llr = fopen("../../../cmake_targets/log/ulsim_ldpc_llr.txt", "w");
  fptr_ldpc = fopen("../../../cmake_targets/log/ulsim_ldpc_output.txt", "w");
  // ===================================
#endif
  //K' = ((A+L)+C*L)/C (TODO: cross check it with their Kprime value)
  int length_dec = lenWithCrc(TB_params->C, TB_params->A);
  //Either CRC 24 or 16
  uint8_t crc_type = crcType(TB_params->C, TB_params->A);
  int no_iteration_ldpc = 2;

  uint32_t num_threads_prepare = 0;

  // calculate required number of jobs
  uint32_t r_while = 0;
  while (r_while < TB_params->C) {
    // calculate number of segments processed in the new job
    uint32_t modulus = (TB_params->C - r_while) % (num_threads_prepare_max - num_threads_prepare);
    uint32_t quotient = (TB_params->C - r_while) / (num_threads_prepare_max - num_threads_prepare);
    uint32_t r_span_max = modulus == 0 ? quotient : quotient + 1;

    // saturate to be sure to not go above C
    uint32_t r_span = TB_params->C - r_while < r_span_max ? TB_params->C - r_while : r_span_max;

    // increment
    num_threads_prepare++;
    r_while += r_span;
  }

  args_fpga_decode_prepare_t arr[num_threads_prepare];
  task_ans_t ans[num_threads_prepare];
  memset(ans, 0, num_threads_prepare * sizeof(task_ans_t));
  thread_info_tm_t t_info = {.buf = (uint8_t *)arr, .len = 0, .cap = num_threads_prepare, .ans = ans};

  // start the prepare jobs
  uint32_t r_remaining = 0;
  for (uint32_t r = 0; r < TB_params->C; r++) {
    nrLDPC_segment_decoding_parameters_t *segment_params = &TB_params->segments[r];
    if (r_remaining == 0) {
      // TODO: int nr_tti_rx = 0;

      args_fpga_decode_prepare_t *args = &((args_fpga_decode_prepare_t *)t_info.buf)[t_info.len];
      DevAssert(t_info.len < t_info.cap);
      args->ans = &t_info.ans[t_info.len];
      t_info.len += 1;

      args->TB_params = TB_params;
      args->multi_indata = &multi_indata[0] + HEADER_SIZE;
      args->no_iteration_ldpc = no_iteration_ldpc;
      args->r_first = r;

      uint32_t modulus = (TB_params->C - r) % (num_threads_prepare_max - num_threads_prepare);
      uint32_t quotient = (TB_params->C - r) / (num_threads_prepare_max - num_threads_prepare);
      uint32_t r_span_max = modulus == 0 ? quotient : quotient + 1;

      uint32_t r_span = TB_params->C - r < r_span_max ? TB_params->C - r : r_span_max;
      args->r_span = r_span;
      args->r_offset = r_offset;
      args->input_CBoffset = input_CBoffset;
      args->Kc = Kc;
      args->Kprime = Kprime;

      r_remaining = r_span;
      //De-concatenation and De-rate matching; not 100 % sure
      task_t t = {.func = &nr_ulsch_FPGA_decoding_prepare_blocks, .args = args};
      pushTpool(ldpc_threadPool, t);

      LOG_D(PHY, "Added %d block(s) to prepare for decoding, in pipe: %d to %d\n", r_span, r, r + r_span - 1);
    }
    r_offset += segment_params->E;
    offset += ((K >> 3) - (TB_params->F >> 3) - ((TB_params->C > 1) ? 3 : 0));
    r_remaining -= 1;
  }

  // reset offset in order to properly fill the output array later
  offset = 0;

  DevAssert(num_threads_prepare == t_info.len);

  // wait for the prepare jobs to complete. meaning all CBs are ready for decoding
  join_task_ans(t_info.ans);

  for (uint32_t job = 0; job < num_threads_prepare; job++) {
    args_fpga_decode_prepare_t *args = &arr[job];
    if (args->no_iteration_ldpc >= TB_params->max_ldpc_iterations)
      no_iteration_ldpc = TB_params->max_ldpc_iterations;
  }

  // launch decode with FPGA
  LOG_I(PHY, "Run the LDPC ------[FPGA version]------\n");
  //==================================================================
  //  Xilinx FPGA LDPC decoding function -> nrLDPC_decoder_FPGA_PYM()
  //==================================================================
  start_meas(&TB_params->segments[0].ts_ldpc_decode);
  nrLDPC_decoder_FPGA_PYM(&multi_indata[0], &multi_outdata[0], dec_conf);
  // printf("Xilinx FPGA -> CB = %d\n", harq_process->C);
  stop_meas(&TB_params->segments[0].ts_ldpc_decode);

  *TB_params->processedSegments = 0;
  for (uint32_t r = 0; r < TB_params->C; r++) {
    // ------------------------------------------------------------
    // --------------------- copy FPGA output ---------------------
    // ------------------------------------------------------------
    nrLDPC_segment_decoding_parameters_t *segment_params = &TB_params->segments[r];
    //Why is crc check the same for all CBs? Is it the CRC check of the TB? Probably this checks just the CRC for the  first CB -> wrong needs to be fixed
    if (check_crc(&multi_outdata[HEADER_SIZE], length_dec, crc_type)) {
#ifdef DEBUG_CRC
      LOG_I(PHY, "Segment %d CRC OK\n", r);
#endif
      no_iteration_ldpc = 2;
    } else {
#ifdef DEBUG_CRC
      LOG_I(PHY, "segment %d CRC NOK\n", r);
#endif
      no_iteration_ldpc = TB_params->max_ldpc_iterations;
    }
    //copy result
    for (int i = 0; i < out_CBoffset; i++) {
      segment_params->c[i] = multi_outdata[i + r * out_CBoffset + HEADER_SIZE];
    }
    segment_params->decodeSuccess = (no_iteration_ldpc < TB_params->max_ldpc_iterations);
    if (segment_params->decodeSuccess) {
      *TB_params->processedSegments = *TB_params->processedSegments + 1;
    }
  }

  return 0;
}

/*!
 * \fn nr_ulsch_FPGA_decoding_prepare_blocks(void *args)
 * \brief prepare blocks for LDPC decoding on FPGA
 *
 * \param args pointer to the arguments of the function in a structure of type args_fpga_decode_prepare_t
 */
void nr_ulsch_FPGA_decoding_prepare_blocks(void *args)
{
  // extract the arguments
  args_fpga_decode_prepare_t *arguments = (args_fpga_decode_prepare_t *)args;

  nrLDPC_TB_decoding_parameters_t *TB_params = arguments->TB_params;

  uint8_t Qm = TB_params->Qm;

  uint8_t BG = TB_params->BG;
  uint8_t rv_index = TB_params->rv_index;
  uint8_t max_ldpc_iterations = TB_params->max_ldpc_iterations;

  uint32_t tbslbrm = TB_params->tbslbrm;
  uint32_t K = TB_params->K;
  uint32_t Z = TB_params->Z;
  uint32_t F = TB_params->F;

  uint32_t C = TB_params->C;

  nrLDPC_segment_decoding_parameters_t *segment_params = &TB_params->segments[0];

  short *ulsch_llr = segment_params->llr;

  uint8_t *multi_indata = arguments->multi_indata;
  int no_iteration_ldpc = arguments->no_iteration_ldpc;
  uint32_t r_first = arguments->r_first;
  uint32_t r_span = arguments->r_span;
  int r_offset = arguments->r_offset;
  int input_CBoffset = arguments->input_CBoffset;
  int Kc = arguments->Kc;
  int Kprime = arguments->Kprime;

  int16_t z[68 * 384 + 16] __attribute__((aligned(16)));
  simde__m128i *pv = (simde__m128i *)&z;

  // the function processes r_span blocks starting from block at index r_first in ulsch_llr
  for (uint32_t r = r_first; r < (r_first + r_span); r++) {
    nrLDPC_segment_decoding_parameters_t *segment_params = &TB_params->segments[r];
    // ----------------------- FPGA pre process ------------------------
    simde__m128i ones = simde_mm_set1_epi8(255); // Generate a vector with all elements set to 255
    simde__m128i *temp_multi_indata = (simde__m128i *)&multi_indata[r * input_CBoffset];
    // -----------------------------------------------------------------

    // code blocks after bit selection in rate matching for LDPC code (38.212 V15.4.0 section 5.4.2.1)
    int16_t harq_e[segment_params->E];
    // -------------------------------------------------------------------------------------------
    // deinterleaving
    // -------------------------------------------------------------------------------------------
    start_meas(&segment_params->ts_deinterleave);
    nr_deinterleaving_ldpc(segment_params->E, Qm, harq_e, ulsch_llr + r_offset);
    stop_meas(&segment_params->ts_deinterleave);
    // -------------------------------------------------------------------------------------------
    // dematching
    // -------------------------------------------------------------------------------------------
    start_meas(&segment_params->ts_rate_unmatch);
    if (nr_rate_matching_ldpc_rx(tbslbrm,
                                 BG,
                                 Z,
                                 segment_params->d,
                                 harq_e,
                                 C,
                                 rv_index,
                                 *segment_params->d_to_be_cleared,
                                 segment_params->E,
                                 F,
                                 K - F - 2 * Z)
        == -1) {
      stop_meas(&segment_params->ts_rate_unmatch);
      LOG_E(PHY, "ulsch_decoding.c: Problem in rate_matching\n");
      no_iteration_ldpc = max_ldpc_iterations;
      arguments->no_iteration_ldpc = no_iteration_ldpc;
      return;
    } else {
      stop_meas(&segment_params->ts_rate_unmatch);
    }

    *segment_params->d_to_be_cleared = false;

    memset(segment_params->c, 0, K >> 3);

    // set first 2*Z_c bits to zeros; are these the punctured bits?
    memset(&z[0], 0, 2 * Z * sizeof(int16_t));
    // set Filler bits
    memset((&z[0] + Kprime), 127, F * sizeof(int16_t));
    // Move coded bits before filler bits
    memcpy((&z[0] + 2 * Z), segment_params->d, (Kprime - 2 * Z) * sizeof(int16_t));
    // skip filler bits
    memcpy((&z[0] + K), segment_params->d + (K - 2 * Z), (Kc * Z - K) * sizeof(int16_t));

    // Saturate coded bits before decoding into 8 bits values
    for (int i = 0, j = 0; j < ((Kc * Z) >> 4); i += 2, j++) {
      temp_multi_indata[j] =
          simde_mm_xor_si128(simde_mm_packs_epi16(pv[i], pv[i + 1]),
                             simde_mm_cmpeq_epi32(ones,
                                                  ones)); // Perform NOT operation and write the result to temp_multi_indata[j]
    }

    // the last bytes before reaching "Kc * harq_process->Z" should not be written 128 bits at a time to avoid overwritting the
    // following block in multi_indata
    simde__m128i tmp =
        simde_mm_xor_si128(simde_mm_packs_epi16(pv[2 * ((Kc * Z) >> 4)], pv[2 * ((Kc * Z) >> 4) + 1]),
                           simde_mm_cmpeq_epi32(ones,
                                                ones)); // Perform NOT operation and write the result to temp_multi_indata[j]
    uint8_t *tmp_p = (uint8_t *)&tmp;
    for (int i = 0, j = ((Kc * Z) & 0xfffffff0); j < Kc * Z; i++, j++) {
      multi_indata[r * input_CBoffset + j] = tmp_p[i];
    }

    r_offset += segment_params->E;
  }

  arguments->no_iteration_ldpc = no_iteration_ldpc;
}
