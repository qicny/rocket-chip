#ifndef _ROCKET_DTM_H
#define _ROCKET_DTM_H

#include <stdint.h>
#include <queue>
#include <semaphore.h>

class dtm_t
{
 public:
  struct req {
    uint32_t addr;
    uint32_t op;
    uint64_t data;
  };

  struct resp {
    uint32_t resp;
    uint64_t data;
  };

  dtm_t();

  void tick(
    bool* req_valid,
    bool  req_ready,
    req*  req_bits,
    bool  resp_valid,
    bool* resp_ready,
    resp  resp_bits
  );

  uint64_t read(uint32_t addr);
  uint64_t write(uint32_t addr, uint64_t data);
  void nop();
  uint32_t run_program(const uint32_t program[], size_t n, size_t result);

  void producer_thread();

 private:
  pthread_t producer;
  sem_t req_produce;
  sem_t req_consume;
  sem_t resp_produce;
  sem_t resp_consume;
  req req_buf;
  resp resp_buf;

  void read_chunk(uint64_t taddr, size_t len, void* dst);
  void write_chunk(uint64_t taddr, size_t len, const void* src);
  size_t chunk_align();
  size_t chunk_max_size();

  bool req_wait;
  req pending_req;
  bool resp_wait;
  uint32_t dminfo;
  uint32_t xlen;

  size_t ram_base() { return 0x400; }
  size_t rom_ret() { return 0x804; }
  size_t ram_words() { return ((dminfo >> 10) & 63) + 1; }
  uint32_t get_xlen();
  uint64_t do_command(dtm_t::req r);
};

#endif
