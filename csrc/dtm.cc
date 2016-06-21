#include "dtm.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#define RV_X(x, s, n) \
  (((x) >> (s)) & ((1 << (n)) - 1))
#define ENCODE_ITYPE_IMM(x) \
  (RV_X(x, 0, 12) << 20)
#define ENCODE_STYPE_IMM(x) \
  ((RV_X(x, 0, 5) << 7) | (RV_X(x, 5, 7) << 25))
#define ENCODE_SBTYPE_IMM(x) \
  ((RV_X(x, 1, 4) << 8) | (RV_X(x, 5, 6) << 25) | (RV_X(x, 11, 1) << 7) | (RV_X(x, 12, 1) << 31))
#define ENCODE_UTYPE_IMM(x) \
  (RV_X(x, 12, 20) << 12)
#define ENCODE_UJTYPE_IMM(x) \
  ((RV_X(x, 1, 10) << 21) | (RV_X(x, 11, 1) << 20) | (RV_X(x, 12, 8) << 12) | (RV_X(x, 20, 1) << 31))

uint64_t dtm_t::do_command(dtm_t::req r)
{
  sem_wait(&req_produce);
  req_buf = r;
  sem_post(&req_consume);

  sem_wait(&resp_consume);
  resp rsp = resp_buf;
  sem_post(&resp_produce);

  assert(rsp.resp == 0);
  return rsp.data;
}

uint64_t dtm_t::read(uint32_t addr)
{
  return do_command((req){addr, 1, 0});
}

uint64_t dtm_t::write(uint32_t addr, uint64_t data)
{
  return do_command((req){addr, 2, data});
}

void dtm_t::nop()
{
  do_command((req){0, 0, 0});
}

uint32_t dtm_t::run_program(const uint32_t program[], size_t n, size_t result)
{
  assert(n <= ram_words());
  assert(result < ram_words());

  uint64_t interrupt_bit = 0x200000000U;
  for (size_t i = 0; i < n; i++)
    write(i, program[i] | (i == n-1 ? interrupt_bit : 0));

  while (true) {
    uint64_t rdata = read(result);
    if (!(rdata & interrupt_bit))
      return (uint32_t)rdata;
  }
}

size_t dtm_t::chunk_align()
{
  return xlen / 8;
}

void dtm_t::read_chunk(uint64_t taddr, size_t len, void* dst)
{
  uint32_t prog[ram_words()];
  uint32_t res[ram_words()];
  int result_word = 2 + (len / (xlen/8)) * 2;
  int addr_word = result_word;
  int prog_words = addr_word + (xlen/32);

  if (xlen == 32) {
    prog[0] = 0x00002403 | ENCODE_ITYPE_IMM(ram_base() + addr_word * 4);
    for (int i = 0; i < len/4; i++) {
      prog[2*i+1] = 0x00042483 | ENCODE_ITYPE_IMM(i * 4);
      prog[2*i+2] = 0x00902023 | ENCODE_STYPE_IMM(ram_base() + (result_word + i) * 4);
    }
  } else {
    prog[0] = 0x00003403 | ENCODE_ITYPE_IMM(ram_base() + addr_word * 4);
    for (int i = 0; i < len/8; i++) {
      prog[2*i+1] = 0x00043483 | ENCODE_ITYPE_IMM(i * 8);
      prog[2*i+2] = 0x00903023 | ENCODE_STYPE_IMM(ram_base() + (result_word/2 + i) * 8);
    }
  }

  prog[result_word - 1] = 0x6f | ENCODE_UJTYPE_IMM(rom_ret() - (ram_base() + (result_word - 1)*4));
  prog[addr_word] = (uint32_t)taddr;
  prog[addr_word + 1] = (uint32_t)(taddr >> 32);

  res[0] = run_program(prog, prog_words, result_word);
  for (int i = 1; i < len/4; i++)
    res[i] = read(result_word + i);
  memcpy(dst, res, len);
}

void dtm_t::write_chunk(uint64_t taddr, size_t len, const void* src)
{
  uint32_t prog[ram_words()];
  int data_word = 2 + (len / (xlen/8)) * 2;
  int addr_word = data_word + len/4;
  int prog_words = addr_word + (xlen/32);

  memcpy(prog + data_word, src, len);

  if (xlen == 32) {
    prog[0] = 0x00002403 | ENCODE_ITYPE_IMM(ram_base() + addr_word * 4);
    for (int i = 0; i < len/4; i++) {
      prog[2*i+1] = 0x00002483 | ENCODE_ITYPE_IMM(ram_base() + (data_word + i) * 4);
      prog[2*i+2] = 0x00942023 | ENCODE_STYPE_IMM(i * 4);
    }
  } else {
    prog[0] = 0x00003403 | ENCODE_ITYPE_IMM(ram_base() + addr_word * 4);
    for (int i = 0; i < len/8; i++) {
      prog[2*i+1] = 0x00003483 | ENCODE_ITYPE_IMM(ram_base() + (data_word/2 + i) * 8);
      prog[2*i+2] = 0x00943023 | ENCODE_STYPE_IMM(i * 8);
    }
  }

  prog[data_word - 1] = 0x6f | ENCODE_UJTYPE_IMM(rom_ret() - (ram_base() + (data_word - 1)*4));
  prog[addr_word] = (uint32_t)taddr;
  prog[addr_word + 1] = (uint32_t)(taddr >> 32);

  run_program(prog, prog_words, data_word);
}

void dtm_t::write_csr(unsigned which, uint64_t data)
{
  uint32_t load = xlen == 64 ? 0x00003403 : 0x00002403;
  int data_word = 4;
  const uint32_t prog[ram_words()] = {
    load | ENCODE_ITYPE_IMM(ram_base() + data_word * 4),
    0x00041073 | ENCODE_ITYPE_IMM(which),
    0x3fc0006f
  };

  prog[data_word] = (uint32_t)data;
  prog[data_word + 1] = (uint32_t)(data >> 32);
  run_program(prog, data_word + (xlen/32), 0);
}

uint64_t dtm_t::read_csr(unsigned which)
{
  uint32_t store = xlen == 64 ? 0x00803023 : 0x00802023;
  int data_word = 4;
  const uint32_t prog[] = {
    0x00002473 | ENCODE_ITYPE_IMM(which),
    store | ENCODE_STYPE_IMM(ram_base() + data_word * 4),
    0x3fc0006f
  };

  uint64_t res = run_program(prog, sizeof(prog)/sizeof(*prog), data_word);
  if (xlen == 64)
    res |= read(data_word + 1) << 32;
}

size_t dtm_t::chunk_max_size()
{
  if (xlen == 32)
    return 4 * ((ram_words() - 4) / 3);
  else
    return 8 * ((ram_words() - 6) / 4);
}

uint32_t dtm_t::get_xlen()
{
  const uint32_t prog[] = {
    0xf1002473,
    0x02000493,
    0x00042413,
    0x008494b3,
    0x40902023,
    0x3f00006f
  };

  uint32_t result = run_program(prog, sizeof(prog)/sizeof(*prog), 0);
  assert(result == 32 || result == 64);
  return result;
}

void* producer_thread_main(void* arg)
{
  ((dtm_t*)arg)->producer_thread();
  return 0;
}

void dtm_t::producer_thread()
{
  dminfo = read(0x11);
  xlen = get_xlen();
  printf("dminfo %x xlen %d\n", dminfo, xlen);

  uint32_t chunk[4];
  read_chunk(0x1000, 16, (void*)chunk);
  for (int i = 0; i < 4; i++)
    printf("%d %x\n", i, chunk[i]);

  chunk[0] = 0xabcdef;
  chunk[1] = 0xfedcba;
  chunk[2] = 0x01234567;
  chunk[3] = 0x76543210;
  write_chunk(0x80000000U, 16, (void*)chunk);
  read_chunk(0x80000000U, 16, (void*)chunk);
  for (int i = 0; i < 4; i++)
    printf("%d %x\n", i, chunk[i]);

  printf("%lx\n", read_csr

  while (true) {
    nop();
  }
}

dtm_t::dtm_t()
{
  req_wait = false;
  resp_wait = false;
  sem_init(&req_produce, 0, 1);
  sem_init(&req_consume, 0, 0);
  sem_init(&resp_produce, 0, 1);
  sem_init(&resp_consume, 0, 0);
  if (pthread_create(&producer, 0, producer_thread_main, this))
    abort();
}

void dtm_t::tick(
  bool*     req_valid,
  bool      req_ready,
  req*      req_bits,
  bool      resp_valid,
  bool*     resp_ready,
  resp      resp_bits)
{
  if (resp_wait) {
    *req_valid = false;
  } else if (!req_wait) {
    *req_valid = true;
    req_wait = true;

    sem_wait(&req_consume);
    pending_req = req_buf;
    sem_post(&req_produce);
  } else if (req_ready) {
    *req_valid = false;
    req_wait = false;
    resp_wait = true;
  } else {
    *req_valid = true;
  }

  *req_bits = pending_req;

  *resp_ready = true;
  if (resp_valid) {
    assert(resp_wait);
    resp_wait = false;

    sem_wait(&resp_produce);
    resp_buf = resp_bits;
    sem_post(&resp_consume);
  }
}
