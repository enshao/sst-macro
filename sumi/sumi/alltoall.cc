#include <sumi/alltoall.h>
#include <sumi/partner_timeout.h>
#include <sumi/transport.h>
#include <sumi/domain.h>
#include <sprockit/output.h>
#include <cstring>

#define divide_by_2_round_up(x) \
  ((x/2) + (x%2))

#define divide_by_2_round_down(x) \
  (x/2)

using namespace sprockit::dbg;

#define SEND_SHUFFLE 0
#define RECV_SHUFFLE 1

namespace sumi
{

SpktRegister("bruck_alltoall", dag_collective, bruck_alltoall_collective);

void
bruck_alltoall_actor::init_buffers(void* dst, void* src)
{
  int log2nproc, num_rounds, nprocs_extra_round;
  compute_tree(log2nproc, midpoint_, num_rounds, nprocs_extra_round);

  if (src){
    //put everything into the dst buffer to begin
    //but we have to shuffle for bruck algorithm
    int offset = dense_me_ * nelems_ * type_size_;
    int total_size = dense_nproc_ * nelems_ * type_size_;
    char* srcPtr = (char*) src;
    char* dstPtr = (char*) dst;
    int copySize = total_size - offset;
    std::memcpy(dstPtr, srcPtr + offset, copySize);
    std::memcpy(dstPtr + copySize, srcPtr, offset);

    int tmp_buffer_size = nelems_ * type_size_ * midpoint_;
    result_buffer_ = my_api_->make_public_buffer(dst, total_size);
    send_buffer_ = my_api_->allocate_public_buffer(tmp_buffer_size);
    recv_buffer_ = my_api_->allocate_public_buffer(tmp_buffer_size);
  }
}

void
bruck_alltoall_actor::finalize_buffers()
{
  if (send_buffer_.ptr){
    int buffer_size = nelems_ * type_size_ * dom_->nproc();
    int tmp_buffer_size = nelems_ * type_size_ * midpoint_;
    my_api_->unmake_public_buffer(result_buffer_, buffer_size);
    my_api_->free_public_buffer(recv_buffer_, tmp_buffer_size);
    my_api_->free_public_buffer(send_buffer_, tmp_buffer_size);
  }
}

void
bruck_alltoall_actor::shuffle(action *ac, void* tmpBuf, void* mainBuf, bool copyToTemp)
{
  int nproc = dense_nproc_;
  char* tmp_buffer = (char*) tmpBuf;
  char* main_buffer = (char*) mainBuf;
  int blocksPerCopy = ac->offset;
  int blockStride = blocksPerCopy*2;
  int tmpBlock = 0;
  for (int mainBlock=ac->offset; mainBlock < nproc; tmpBlock += blocksPerCopy, mainBlock += blockStride){
    int remainingBlocks = nproc - mainBlock;
    int numCopyBlocks = std::min(blocksPerCopy, remainingBlocks);
    int copySize = numCopyBlocks * nelems_ * type_size_;
    void* tmp = tmp_buffer + tmpBlock*nelems_*type_size_;
    void* main = main_buffer + mainBlock*nelems_*type_size_;
    if (copyToTemp){
      ::memcpy(tmp, main, copySize);
    } else {
      ::memcpy(main, tmp, copySize);
    }
  }
}

void
bruck_alltoall_actor::start_shuffle(action *ac)
{
  if (result_buffer_.ptr == 0) return;

  if (ac->partner == SEND_SHUFFLE){
    //shuffle to get ready for a send
    //shuffle from result_buffer into send_buffer
    shuffle(ac, send_buffer_, result_buffer_, true/*copy to temp send buffer*/);
  } else {
    shuffle(ac, recv_buffer_, result_buffer_, false/*copy from temp recv into result*/);
  }

}

void
bruck_alltoall_actor::init_dag()
{
  int log2nproc, num_rounds, nprocs_extra_round;
  compute_tree(log2nproc, midpoint_, num_rounds, nprocs_extra_round);

  int partnerGap = 1;
  int me = dense_me_;
  int nproc = dense_nproc_;
  action* prev_shuffle = 0;
  if (nprocs_extra_round) ++num_rounds;

  for (int round=0; round < num_rounds; ++round){
    int up_partner = (me + partnerGap) % nproc;
    int down_partner = (me - partnerGap + nproc) % nproc;
    int intervalSendSize = partnerGap;
    int elemStride = partnerGap*2;
    int bruckIntervalSize = elemStride;

    int sendWindowSize = nproc - partnerGap;
    int numBruckIntervals = sendWindowSize / bruckIntervalSize;
    int numSendBlocks = numBruckIntervals * intervalSendSize;
    int remainder = sendWindowSize % bruckIntervalSize;
    int extraSendBlocks = std::min(intervalSendSize, remainder);
    numSendBlocks += extraSendBlocks;

    action* send_shuffle = new shuffle_action(round, SEND_SHUFFLE);
    action* recv_shuffle = new shuffle_action(round, RECV_SHUFFLE);
    action* send = new send_action(round, up_partner);
    action* recv = new recv_action(round, down_partner);

    int nelemsRound = numSendBlocks * nelems_;

    send->offset = 0;
    send->nelems = nelemsRound;

    recv->offset = 0;
    recv->nelems = nelemsRound;
    recv->recv_type = action::temp;

    send_shuffle->offset = partnerGap;
    recv_shuffle->offset = partnerGap;
    send_shuffle->nelems = nelemsRound;
    recv_shuffle->nelems = nelemsRound;

    if (prev_shuffle){
      add_dependency(prev_shuffle, send_shuffle);
      add_dependency(prev_shuffle, send_shuffle);
    } else {
      add_initial_action(send_shuffle);
    }

    add_dependency(send_shuffle, send);
    add_dependency(send_shuffle, recv);

    add_dependency(send, recv_shuffle);
    add_dependency(recv, recv_shuffle);

    prev_shuffle = recv_shuffle;
    partnerGap *= 2;
  }
}

void
bruck_alltoall_actor::buffer_action(void *dst_buffer, void *msg_buffer, action* ac)
{
  std::memcpy(dst_buffer, msg_buffer, ac->nelems * type_size_);
}

void
bruck_alltoall_actor::finalize()
{
  if (result_buffer_ == 0){
    return;
  }

  int total_size = dense_nproc_ * nelems_ * type_size_;
  int block_size = nelems_ * type_size_;
  char* tmp = new char[total_size];
  char* result = (char*) result_buffer_.ptr;
  for (int i=0; i < dense_nproc_; ++i){
    char* src = result + i*block_size;
    int dst_index = (dense_me_ + dense_nproc_ - i) % dense_nproc_;
    char* dst = tmp + dst_index*block_size;
    ::memcpy(dst, src, block_size);
  }

  ::memcpy(result, tmp, total_size);

  do_sumi_debug_print("final result buf",
    rank_str().c_str(), dense_me_,
    -1,
    0, nelems_*dense_nproc_,
    type_size_,
    result_buffer_.ptr);

  delete[] tmp;
}


}

