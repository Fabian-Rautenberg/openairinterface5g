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
#define USE_PARITY_OPTIMIZATION (true)
#define USE_OUTPUT_PARALLELIZATION (false)
#define DO_INTERNAL_TIME_MEASUREMENT (true)
#define USE_EXACT_BG (true)  


#include "nfapi/open-nFAPI/nfapi/public_inc/nfapi_interface.h"
#include "nfapi/open-nFAPI/nfapi/public_inc/nfapi_nr_interface.h"

#include "PHY/CODING/nrLDPC_coding/nrLDPC_coding_interface.h"

#include "xdma_diag.h"
#if DO_INTERNAL_TIME_MEASUREMENT
#include "common/utils/var_array.h"
#include "SIMULATION/LTE_PHY/common_sim.h"
#endif


// Global var to limit the rework of the dirty legacy code
int num_threads_prepare_max = 0;
char *user_device = NULL;
char *enc_read_device = NULL;
char *enc_write_device = NULL;
char *dec_read_device = NULL;
char *dec_write_device = NULL;
#if DO_INTERNAL_TIME_MEASUREMENT
//Time measurements declaration begin
typedef struct internal_time_stats_s {
  time_stats_t ts_total_decoding_prepare_time;
  time_stats_t ts_prepare_copying_time;
  time_stats_t ts_total_decoding_post_time;

  time_stats_t ts_deinterleaving_time;
  time_stats_t ts_rate_dematching_time;

  time_stats_t ts_total_decoding_time;
  time_stats_t ts_h2c_latency;
  time_stats_t ts_c2h_time;
  time_stats_t ts_h2c_time;
  time_stats_t ts_hw_dec_latency; //< valid to last
  
  time_stats_t total_process_tb_time;

  bool valid;
  size_t numb_of_decoder_iter;
  uint32_t coderate;
} internal_time_stats_t;
#define NUMB_OF_TOTAL_TIME_POINTS (100U) //< assuming SNR step size 0.2, SNR range is 10 -> 10/0.2 = 50
#define NUMB_OF_MAX_RETRANSMISSION (4U)
#define NUMBER_OF_TRIALS_PER_SNR (100U)
static internal_time_stats_t internal_time_stats[NUMB_OF_TOTAL_TIME_POINTS][NUMB_OF_MAX_RETRANSMISSION];
static uint32_t timer_idx = 0;
static uint32_t K4MS = 0; 
static uint32_t Qm4MS = 0;
static uint32_t MCS4MS= 0;
static uint32_t CBs4MS = 0;
//Time measurements declaration end
#endif


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
  int Kb;
  task_ans_t *ans; /*!< pointer to the answer that is used by thread pool to detect job completion */
#if DO_INTERNAL_TIME_MEASUREMENT
  time_stats_t* ts_copying_to_FPGA_buff;/*!< time needed to copy to FPGA buff */
#endif
} args_fpga_decode_prepare_t;

typedef struct args_fpga_post_decode_s {
  nrLDPC_TB_decoding_parameters_t *TB_params; /*!< transport blocks parameters */

  uint32_t r_first; /*!< index of the first block to be prepared within this function */
  uint32_t r_span; /*!< number of blocks to be prepared within this function */
  int output_CBoffset; /*!< */
  uint8_t *multi_outdata; /*!< pointer to the head of the block destination array that is received from FPGA decoding */
  int K;
  int length_dec;
  uint8_t crc_type;
  task_ans_t *ans; /*!< pointer to the answer that is used by thread pool to detect job completion */
} args_fpga_post_decode_t;

int32_t nrLDPC_coding_init(void);
int32_t nrLDPC_coding_shutdown(void);
int32_t nrLDPC_coding_decoder(nrLDPC_slot_decoding_parameters_t *slot_params, int frame_rx, int slot_rx);
// int32_t nrLDPC_coding_encoder(void);
int decoder_xdma(nrLDPC_TB_decoding_parameters_t *TB_params, int frame_rx, int slot_rx, tpool_t *ldpc_threadPool);
void nr_ulsch_FPGA_decoding_prepare_blocks(void *args);
void nr_ulsch_FPGA_post_decoding(void *args);

static inline size_t get_number_of_parity_bits(const bool d_to_clear, const uint32_t E, const uint32_t Z, const uint32_t Kprime, const uint8_t BG, const bool padded);
static uint32_t get_CB_offset(const bool d_to_clear, const uint32_t Z, const uint32_t Kc, const uint32_t E, const uint32_t F);
static inline size_t get_Z_padding(const size_t nbits, const uint32_t Z);
static inline simde__m128i reverse_bits_8x16(simde__m128i* x);

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
  dec_conf.dec_write_time = &time_stats->llr2CnProcBuf;
  dec_conf.dec_read_time = &time_stats->cn2bnProcBuf;
  dec_conf.hw_dec_time = &time_stats->bnProc;
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
  dec_conf.SetIdx = 12;
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
  const int F = K - p_decParams->Kprime;
  const int punctured_bits = 2 * p_decParams->Z; 
  const int8_t max_level = 120;
  start_meas(&time_stats->llr2llrProcBuf);
  //copy all LLRs in internal buffer starting at HEADER_SIZE
  start_meas(&time_stats->total);
  //copy punctured bits
  memcpy(&buffer_in[HEADER_SIZE], p_llr, punctured_bits * sizeof(*p_llr));
  //copy information bits
  for(int i = punctured_bits; i < p_decParams->Kprime; ++i)
  {
    if(p_llr[i] > max_level)
      buffer_in[HEADER_SIZE + i] = max_level;
    else if(p_llr[i] < -max_level)
      buffer_in[HEADER_SIZE + i] = -max_level;
    else
      buffer_in[HEADER_SIZE + i] = p_llr[i];
  }
  //set filler bits
  memset(&buffer_in[HEADER_SIZE + p_decParams->Kprime], max_level, F);
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
  stop_meas(&time_stats->llr2llrProcBuf);
  start_meas(&time_stats->llr2bit);
  int32_t niter = nrLDPC_decoder_FPGA_PYM((uint8_t*)&buffer_in[0], (uint8_t*)&buffer_out[0], dec_conf);
  stop_meas(&time_stats->llr2bit);
  //calculate number of 8 bit units to copy
  const int cK = (Kb * p_decParams->Z + 7) / 8; 
  start_meas(&time_stats->llrRes2llrOut);
  //copy into out buffer and reverse bits
  for(int i = 0; i < cK; ++i)
  {
    p_out[i] = (int8_t)reverse_8bit((uint8_t)buffer_out[HEADER_SIZE + i]);
  }
  //set the remaining bits to zero
  const int rem = (K + 7) / 8 - cK;
  memset(p_out + cK, 0, rem);
  stop_meas(&time_stats->llrRes2llrOut);
  stop_meas(&time_stats->total);
  if(p_decParams->check_crc != NULL)
  { 
    const bool crc_valid = p_decParams->check_crc((uint8_t*)p_out, p_decParams->Kprime, p_decParams->crc_type);
    if (!crc_valid) 
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





int32_t nrLDPC_coding_init(void)
{
  paramdef_t LoaderParams[] = {
      {"num_threads_prepare", NULL, 0, .iptr = &num_threads_prepare_max, .defintval = 0, TYPE_INT, 0, NULL},
      {"user_device", NULL, 0, .strptr = &user_device, .defstrval = DEVICE_NAME_DEFAULT_USER, TYPE_STRING, 0, NULL},
      {"enc_read_device", NULL, 0, .strptr = &enc_read_device, .defstrval = DEVICE_NAME_DEFAULT_ENC_READ, TYPE_STRING, 0, NULL},
      {"enc_write_device", NULL, 0, .strptr = &enc_write_device, .defstrval = DEVICE_NAME_DEFAULT_ENC_WRITE, TYPE_STRING, 0, NULL},
      {"dec_read_device", NULL, 0, .strptr = &dec_read_device, .defstrval = DEVICE_NAME_DEFAULT_DEC_READ, TYPE_STRING, 0, NULL},
      {"dec_write_device", NULL, 0, .strptr = &dec_write_device, .defstrval = DEVICE_NAME_DEFAULT_DEC_WRITE, TYPE_STRING, 0, NULL}};
  //config_get(config_get_if(), LoaderParams, sizeofArray(LoaderParams), "nrLDPC_coding_xdma");
  //AssertFatal(num_threads_prepare_max != 0, "nrLDPC_coding_xdma.num_threads_prepare was not provided");
#if DO_INTERNAL_TIME_MEASUREMENT  
  memset(&internal_time_stats[0][0], 0, sizeof(internal_time_stats));
  timer_idx = 0;
#endif
  return LDPCinit();
}

int32_t nrLDPC_coding_shutdown(void)
{
#if DO_INTERNAL_TIME_MEASUREMENT
  varArray_t* vr = initVarArray(1, sizeof(double));
  *((double*)dataArray(vr)) = 0;
  vr->size++;
  printf("K %u, Qm: %u, MCS: %u, CBs: %u\n", K4MS, Qm4MS, MCS4MS, CBs4MS);
  //Output internal timestats to file
  for(size_t i = 0; i < NUMB_OF_TOTAL_TIME_POINTS; ++i)
  {
    printf("Total measurement IDX %lu:\n", i);
    int invalid_cnt = 0;
    for(size_t j = 0; j < NUMB_OF_MAX_RETRANSMISSION; ++j)
    {
      if(!internal_time_stats[i][j].valid)
      {
        invalid_cnt++;
        continue;
      }
      printf("--------------------------------------------------------------------------------------------\n");
      printf("Retransmission idx: %lu\n", j);
      printf("Number of LDPC decoder iteration done: %lu\n", internal_time_stats[i][j].numb_of_decoder_iter);
      printf("Code rate: %lu\n", internal_time_stats[i][j].coderate);
      printStatIndent(&internal_time_stats[i][j].total_process_tb_time, "Total TB process time");
      printDistribution(&internal_time_stats[i][j].total_process_tb_time, vr, "Total TB process time distribution");
      printStatIndent2(&internal_time_stats[i][j].ts_total_decoding_time, "Total decoding time for all CBs");
      printDistribution(&internal_time_stats[i][j].ts_total_decoding_time, vr, "Total decoding time for all CBs distribution");
      printStatIndent3(&internal_time_stats[i][j].ts_c2h_time, "C2H transfer time for all CBs");
      printDistribution(&internal_time_stats[i][j].ts_c2h_time, vr, "C2H transfer time for all CBs distribution");
      printStatIndent3(&internal_time_stats[i][j].ts_h2c_time, "H2C transfer time for all CBs");
      printDistribution(&internal_time_stats[i][j].ts_h2c_time, vr, "H2C transfer time for all CBs distribution");
      printStatIndent3(&internal_time_stats[i][j].ts_h2c_latency, "H2C latency");
      printDistribution(&internal_time_stats[i][j].ts_h2c_latency, vr, "H2C latency distribution");
      printStatIndent3(&internal_time_stats[i][j].ts_hw_dec_latency, "HW dec latency (valid to last) for all CBs");
      printDistribution(&internal_time_stats[i][j].ts_hw_dec_latency, vr, "HW dec latency (valid to last) for all CBs distribution");
      printStatIndent2(&internal_time_stats[i][j].ts_total_decoding_prepare_time, "Total prepare time for all CBs");
      printDistribution(&internal_time_stats[i][j].ts_total_decoding_prepare_time, vr, "Total prepare time for all CBs distribution");
      printStatIndent3(&internal_time_stats[i][j].ts_deinterleaving_time, "Deinterleaving per CB");
      printDistribution(&internal_time_stats[i][j].ts_deinterleaving_time, vr, "Deinterleaving per CB distribution");
      printStatIndent3(&internal_time_stats[i][j].ts_rate_dematching_time, "Rate dematching per CB");
      printDistribution(&internal_time_stats[i][j].ts_rate_dematching_time, vr, "Rate dematching per CB distribution");
      printStatIndent3(&internal_time_stats[i][j].ts_prepare_copying_time, "Prepare copying time per CB");
      printDistribution(&internal_time_stats[i][j].ts_prepare_copying_time, vr, "Prepare copying time per CB distribution");
      printStatIndent2(&internal_time_stats[i][j].ts_total_decoding_post_time, "Post decoding time for all CBs");
      printDistribution(&internal_time_stats[i][j].ts_total_decoding_post_time, vr, "Post decoding time for all CBs distribution");
    }
    printf("*********************************************************************************************\n");
    if(invalid_cnt == NUMB_OF_MAX_RETRANSMISSION)
      break;
  }
  free(vr);
#endif
  return LDPCshutdown();
}

int32_t nrLDPC_coding_decoder(nrLDPC_slot_decoding_parameters_t *slot_params, int frame_rx, int slot_rx)
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
  DevAssert(TB_params->C <= MAX_CB);
  const uint32_t K = TB_params->K;
  //numb of columns in the BG (52 for BG2 and 68 for BG1) -> number of coded words
  const int Kc = TB_params->BG == 2 ? 52 : 68;
  int r_offset = 0;
  //number of true information bits; could also include CRC of CB
  const int Kprime = K - TB_params->F;
  // FPGA parameter preprocessing
  #define MAX_INPUT_FPGA_SIZE CEIL_UP_16B((OAI_UL_LDPC_MAX_NUM_LLR + HEADER_SIZE) * MAX_CB)
  #define MAX_OUTPUT_FPGA_SIZE CEIL_UP_16B((MAX_CB_SIZE_IN_BYTE_UNITS + HEADER_SIZE) * MAX_CB)
  static uint8_t multi_indata[MAX_INPUT_FPGA_SIZE] __attribute__((aligned(PAGE_SIZE))); // FPGA input data
  static uint8_t multi_outdata[MAX_OUTPUT_FPGA_SIZE] __attribute__((aligned(PAGE_SIZE))); // FPGA output data
  //maximum possible K_b value
  int bg_len = TB_params->BG == 1 ? 22 : 10;

  int input_CBoffset = 0;

  DecIFConf dec_conf = {0};
#if DO_INTERNAL_TIME_MEASUREMENT
  //setup timer stuff
  static uint32_t local_trial_cntr = 0;
  const uint32_t current_timer_idx = timer_idx;
  const uint32_t current_retransmission_idx = TB_params->rv_index;
  internal_time_stats_t* current_time_stat = &internal_time_stats[current_timer_idx][current_retransmission_idx];
  start_meas(&current_time_stat->total_process_tb_time);
  current_time_stat->valid = true;
  dec_conf.dec_write_time = &current_time_stat->ts_h2c_time;
  dec_conf.dec_read_time  = &current_time_stat->ts_c2h_time;
  dec_conf.hw_dec_time    = &current_time_stat->ts_hw_dec_latency;
  dec_conf.h2c_latency    = &current_time_stat->ts_h2c_latency;
  time_stats_t prepare_copying_time[MAX_CB] = {};
  local_trial_cntr += TB_params->rv_index == 0; //< assuming first transmission starts with rv_index 0 and it isn't repeated anymore
  if(local_trial_cntr == NUMBER_OF_TRIALS_PER_SNR)
  {
    local_trial_cntr = 0;
    timer_idx++;
  }
  K4MS = K;
  Qm4MS = TB_params->Qm;
  MCS4MS = TB_params->mcs;
  CBs4MS = TB_params->C;
#endif
  dec_conf.Zc = TB_params->Z;
  int Kb = 0;
  if(TB_params->BG == 1)
  {
    Kb = 22;
    dec_conf.BG = 1;
  }
  else //second BG
  {
#if USE_EXACT_BG 
    //The following has to be valid K_b * Z_c >= K'
    if(6 * TB_params->Z >= Kprime)
    {
      Kb = 6;
      dec_conf.BG = 5;
    }
    else if(8 * TB_params->Z >= Kprime)
    {
      Kb = 8;
      dec_conf.BG = 4;
    }
    else if(9 * TB_params->Z >= Kprime)
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
  //Z_c * (10 or 22) is K; Calculate the number of bits in one CB
  int out_CBoffset = dec_conf.Zc * bg_len + HEADER_SIZE * 8;
  out_CBoffset = CEIL_UP(out_CBoffset, 128);
  //conv to 8 bit units
  out_CBoffset /= 8;
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
  const int length_dec = lenWithCrc(TB_params->C, TB_params->A);
  //Either CRC 24 or 16
  const uint8_t crc_type = crcType(TB_params->C, TB_params->A);
  int no_iteration_ldpc = 2;
  
  num_threads_prepare_max = ldpc_threadPool->len_thr;
  uint32_t num_threads_prepare = 0;
  uint32_t r_spans[MAX_CB] = {};
  //calculate required number of jobs
  if(num_threads_prepare_max == 0)
  {
    r_spans[0] = TB_params->C;
    num_threads_prepare = 0;
  }
  else
  {
    uint32_t r_while = 0;
    while (r_while < TB_params->C && num_threads_prepare_max > 0) {
      // calculate number of segments processed in the new job
      const uint32_t r_rem = TB_params->C - r_while;
      const uint32_t t_rem = num_threads_prepare_max - num_threads_prepare;
      const uint32_t modulus = r_rem % t_rem;
      const uint32_t quotient = r_rem / t_rem;
      const uint32_t r_span_max = modulus == 0 ? quotient : quotient + 1;
  
      // saturate to be sure to not go above C
      const uint32_t r_span = min(r_rem, r_span_max);
      r_spans[r_while] = r_span;
      // increment
      num_threads_prepare++;
      r_while += r_span;
    }
  }
  const size_t arr_size = max(num_threads_prepare, 1);
  args_fpga_decode_prepare_t arr[arr_size];
  task_ans_t ans;
  init_task_ans(&ans, arr_size);
  thread_info_tm_t t_info = {.buf = (uint8_t *)arr, .len = 0, .cap = arr_size, .ans = &ans};
#if DO_INTERNAL_TIME_MEASUREMENT
  start_meas(&current_time_stat->ts_total_decoding_prepare_time);
#endif
  // start the prepare jobs
  for (uint32_t r = 0; r < TB_params->C; /*r+=r_span*/) {
    args_fpga_decode_prepare_t *args = &((args_fpga_decode_prepare_t *)t_info.buf)[t_info.len];
    DevAssert(t_info.len < t_info.cap);
    args->ans = t_info.ans;
    t_info.len += 1;

    args->TB_params = TB_params;
    args->multi_indata = &multi_indata[0];
    args->no_iteration_ldpc = no_iteration_ldpc;
    args->r_first = r;
    const uint32_t r_span = r_spans[r]; 
    args->r_span = r_span;
    args->r_offset = r_offset;
    args->input_CBoffset = input_CBoffset;
    args->Kc = Kc;
    args->Kprime = Kprime;
    args->Kb = Kb;
#if DO_INTERNAL_TIME_MEASUREMENT
    args->ts_copying_to_FPGA_buff = &prepare_copying_time[0];
#endif

    //add offset before starting threads, because d_to_be_cleared can be reseted in threads
    for(size_t current_r = r; current_r < (r + r_span); ++current_r)
    {
      const nrLDPC_segment_decoding_parameters_t *segment_params = &TB_params->segments[current_r];
      input_CBoffset += get_CB_offset(*segment_params->d_to_be_cleared, TB_params->Z, Kc, segment_params->E, TB_params->F);
      dec_conf.numb_of_parity_bits_per_CB[current_r] = get_number_of_parity_bits(*segment_params->d_to_be_cleared, segment_params->E, TB_params->Z, Kprime, TB_params->BG, true);
      r_offset += segment_params->E;
    }
    task_t t = {.func = &nr_ulsch_FPGA_decoding_prepare_blocks, .args = args};
    pushTpool(ldpc_threadPool, t);
    r += r_span;
    LOG_D(PHY, "Added %d block(s) to prepare for decoding, in pipe: %d to %d\n", r_span, r, r + r_span - 1);
  }

  DevAssert(arr_size == t_info.len);

  // wait for the prepare jobs to complete. meaning all CBs are ready for decoding
  join_task_ans(t_info.ans);
#if DO_INTERNAL_TIME_MEASUREMENT
  stop_meas(&current_time_stat->ts_total_decoding_prepare_time);
  for(uint32_t r = 0; r < TB_params->C; r++)
  {
    merge_meas(&current_time_stat->ts_prepare_copying_time, &prepare_copying_time[r]);
    merge_meas(&current_time_stat->ts_rate_dematching_time, &TB_params->segments[r].ts_rate_unmatch);
    merge_meas(&current_time_stat->ts_deinterleaving_time, &TB_params->segments[r].ts_deinterleave);
    current_time_stat->coderate = TB_params->segments[r].R;
  }
#endif
  // launch decode with FPGA
  LOG_I(PHY, "Run the LDPC ------[FPGA version]------\n");
  //==================================================================
  //  Xilinx FPGA LDPC decoding function -> nrLDPC_decoder_FPGA_PYM()
  //==================================================================
  start_meas(&TB_params->segments[0].ts_ldpc_decode);
  const int numb_of_iter = nrLDPC_decoder_FPGA_PYM(&multi_indata[0], &multi_outdata[0], dec_conf);
  stop_meas(&TB_params->segments[0].ts_ldpc_decode);
#if DO_INTERNAL_TIME_MEASUREMENT
  merge_meas(&current_time_stat->ts_total_decoding_time, &TB_params->segments[0].ts_ldpc_decode);
  current_time_stat->numb_of_decoder_iter = numb_of_iter;
#endif
  //Copy to external buffer using the threadpool
  init_task_ans(&ans, arr_size);
  args_fpga_post_decode_t post_decode_args[MAX_CB];
#if DO_INTERNAL_TIME_MEASUREMENT
  start_meas(&current_time_stat->ts_total_decoding_post_time);
#endif
  for (uint32_t r = 0; r < TB_params->C; /*r+=r_span*/) 
  {
    const uint32_t r_span = r_spans[r];
    post_decode_args[r].ans = &ans;
    post_decode_args[r].r_first = r;
    post_decode_args[r].crc_type = crc_type;
    post_decode_args[r].length_dec = length_dec;
    post_decode_args[r].K = K;
    post_decode_args[r].r_span = r_span;
    post_decode_args[r].TB_params = TB_params;
    post_decode_args[r].output_CBoffset = out_CBoffset;
    post_decode_args[r].multi_outdata = &multi_outdata[0];
    task_t t = {.func = &nr_ulsch_FPGA_post_decoding, .args = &post_decode_args[r]};
    #if USE_OUTPUT_PARALLELIZATION
    pushTpool(ldpc_threadPool, t);
    #else
    tpool_t tmp_pool = {.len_thr = 0};  
    pushTpool(&tmp_pool, t);
    #endif
    r += r_span;
  }
  //wait for threads to be completed
  join_task_ans(&ans);
#if DO_INTERNAL_TIME_MEASUREMENT
  stop_meas(&current_time_stat->ts_total_decoding_post_time);
#endif

  //calculate the number of processed segments
  *TB_params->processedSegments = 0;
  for(uint32_t r = 0; r < TB_params->C; ++r)
  {
    *TB_params->processedSegments += TB_params->segments[r].decodeSuccess;
  }
#if DO_INTERNAL_TIME_MEASUREMENT
  stop_meas(&current_time_stat->total_process_tb_time);
#endif 
  return 0;
}

static uint32_t get_CB_offset(const bool d_to_clear, const uint32_t Z, const uint32_t Kc, const uint32_t E, const uint32_t F)
{
  uint32_t offset = 0;
  if(d_to_clear && USE_PARITY_OPTIMIZATION)
  {
    //partial parity bits are sent to HW
    const size_t nbits = E + F + 2 * Z;
    const size_t padding = get_Z_padding(nbits, Z); //inorder to make parity bits a multiple of Z 
    offset = padding + nbits + HEADER_SIZE; 
  }
  else
  {
    offset = Z * Kc + HEADER_SIZE; 
  }
  offset = CEIL_UP_16B(offset);
  return offset;
}

static inline size_t get_Z_padding(const size_t nbits, const uint32_t Z)
{
  return GET_PADDING(nbits, Z);
}

static inline size_t get_number_of_parity_bits(const bool d_to_clear, const uint32_t E, const uint32_t Z, const uint32_t Kprime, const uint8_t BG, const bool padded)
{
  size_t numb_of_parity_bits = 0;
  if(d_to_clear && USE_PARITY_OPTIMIZATION)
  {
    numb_of_parity_bits = E - (Kprime - 2 * Z);
    const size_t padding = get_Z_padding(numb_of_parity_bits, Z);
    //make numb of parity bits a multiple of Z
    numb_of_parity_bits += padded * padding;
  }
  else
  {
    numb_of_parity_bits = BG == 1 ? 46 * Z : 42 * Z;
  }
  return numb_of_parity_bits;
}

static inline simde__m128i reverse_bits_8x16(simde__m128i* x) {

  const simde__m128i lut = simde_mm_setr_epi8(
      0x0, 0x8, 0x4, 0xC,
      0x2, 0xA, 0x6, 0xE,
      0x1, 0x9, 0x5, 0xD,
      0x3, 0xB, 0x7, 0xF
  );

  const simde__m128i mask = simde_mm_set1_epi8(0x0F);

  simde__m128i lo = simde_mm_and_si128(*x, mask);
  simde__m128i hi = simde_mm_and_si128(
      simde_mm_srli_epi16(*x, 4),
      mask
  );

  lo = simde_mm_shuffle_epi8(lut, lo);
  hi = simde_mm_shuffle_epi8(lut, hi);

  return simde_mm_or_si128(
      simde_mm_slli_epi16(lo, 4),
      hi
  );
}

void nr_ulsch_FPGA_post_decoding(void *args)
{
  args_fpga_post_decode_t *arguments = (args_fpga_post_decode_t *)args;
  const uint32_t r_end = arguments->r_first + arguments->r_span;
  const int K = arguments->K;
  for (uint32_t r = arguments->r_first; r < r_end; r++) 
  {
    nrLDPC_segment_decoding_parameters_t *segment_params = &arguments->TB_params->segments[r];
    //copy result bits need to be reversed
    const size_t cK = (K + 7) / 8;
    const size_t numb_of_16B_units = cK / 16;
    const uint8_t* CB_out_ptr = &arguments->multi_outdata[r * arguments->output_CBoffset + HEADER_SIZE];
    const simde__m128i* ptr = (const simde__m128i *)CB_out_ptr; 
    simde__m128i* out_ptr = (simde__m128i *)&segment_params->c[0];
    for (int i = 0; i < numb_of_16B_units; i++)
    {
      const simde__m128i reversed16B = reverse_bits_8x16(&ptr[i]);
      out_ptr[i] = reversed16B;
      //legacy reversing
      //segment_params->c[i] = reverse_8bit(multi_outdata[i + r * out_CBoffset + HEADER_SIZE]);
    }
    //copy and reserving remaining bytes
    const uint32_t rem = cK % 16;
    for(int i = cK - rem; i < cK; ++i)
    {
      const uint8_t reversed = reverse_8bit(CB_out_ptr[i]); 
      segment_params->c[i] = reversed;
    }
    const bool crc_successful = check_crc(segment_params->c, arguments->length_dec, arguments->crc_type);  
    segment_params->decodeSuccess = crc_successful; 
  }
  completed_task_ans(arguments->ans);
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
  const int Kb = arguments->Kb;
  const int KbZ = Kb * Z;
  //Filler bits regarding Kb value
  const int FF = KbZ - Kprime;
  int16_t z[68 * 384 + 16] __attribute__((aligned(16)));
  simde__m128i *pv = (simde__m128i *)&z;
  // the function processes r_span blocks starting from block at index r_first in ulsch_llr
  for (uint32_t r = r_first; r < (r_first + r_span); r++) {
    nrLDPC_segment_decoding_parameters_t *segment_params = &TB_params->segments[r];
    const size_t offset = get_CB_offset(*segment_params->d_to_be_cleared, Z, Kc, segment_params->E, F);
    // ----------------------- FPGA pre process ------------------------
    simde__m128i *temp_multi_indata = (simde__m128i *)&multi_indata[input_CBoffset + HEADER_SIZE];
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
    //Check, if this error occurs 
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
      completed_task_ans(arguments->ans);
      return;
    } else {
      stop_meas(&segment_params->ts_rate_unmatch);
    }

    memset(segment_params->c, 0, K >> 3);

    // set first 2*Z_c bits to zeros; are these the punctured bits?
    memset(&z[0], 0, 2 * Z * sizeof(int16_t));
    // set Filler bits
    memset((&z[0] + Kprime), 120, FF * sizeof(int16_t));
    // Move coded bits before filler bits
    memcpy((&z[0] + 2 * Z), segment_params->d, (Kprime - 2 * Z) * sizeof(int16_t));
    const uint32_t numb_of_parity_bits = Kc * Z - K;
    // skip filler bits, set paraity bits
    memcpy((&z[0] + KbZ), segment_params->d + (K - 2 * Z), numb_of_parity_bits * sizeof(int16_t));

    const simde__m128i max_val =  simde_mm_set1_epi8(120);
    const simde__m128i min_val =  simde_mm_set1_epi8(-120);
    size_t numb_to_copy = KbZ + numb_of_parity_bits;
    if(*segment_params->d_to_be_cleared && USE_PARITY_OPTIMIZATION)
    {
      numb_to_copy = 2 * Z + segment_params->E + FF;
      numb_to_copy += get_Z_padding(numb_to_copy, Z);
    }
    numb_to_copy = CEIL_UP_16B(numb_to_copy); 
#if DO_INTERNAL_TIME_MEASUREMENT
    start_meas(&arguments->ts_copying_to_FPGA_buff[r]);
#endif
    // Saturate coded bits before decoding into 8 bits values
    for (int i = 0, j = 0; j < ((numb_to_copy) >> 4); i += 2, j++) {
      temp_multi_indata[j] = simde_mm_packs_epi16(pv[i], pv[i + 1]);  // transform 16 bit values to 8 bit values
      //saturate from -120 to +120
      temp_multi_indata[j] = simde_mm_max_epi8(temp_multi_indata[j], min_val);
      temp_multi_indata[j] = simde_mm_min_epi8(temp_multi_indata[j], max_val);
    }
#if DO_INTERNAL_TIME_MEASUREMENT
    stop_meas(&arguments->ts_copying_to_FPGA_buff[r]);
#endif
    r_offset += segment_params->E;
    *segment_params->d_to_be_cleared = false;
    input_CBoffset += offset;
  }

  arguments->no_iteration_ldpc = no_iteration_ldpc;
  completed_task_ans(arguments->ans);
}
