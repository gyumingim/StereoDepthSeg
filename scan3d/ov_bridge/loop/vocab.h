// ovloop vocab.h (2026-10-26, branch ovloop-20261026): small bag-of-words vocabulary tree for 256-bit ORB descriptors
// (DBoW2-style hierarchical k-majority tree, TF-IDF weights, L1 score) + an inverted-file database.
// Written here because no DBoW2/DBoW3 or ORB vocabulary exists offline on this PC (no downloads allowed); the
// vocabulary is trained on our own recordings with ovloop_tool (see loop/README in runs/ovloop_20261026).
#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace ovloop {

constexpr int kDescBytes = 32;  // ORB 256 bit

inline int hamming32(const uint8_t *a, const uint8_t *b) {
  const uint64_t *x = reinterpret_cast<const uint64_t *>(a), *y = reinterpret_cast<const uint64_t *>(b);
  return __builtin_popcountll(x[0] ^ y[0]) + __builtin_popcountll(x[1] ^ y[1]) + __builtin_popcountll(x[2] ^ y[2]) +
         __builtin_popcountll(x[3] ^ y[3]);
}

// sparse BoW vector: sorted (word, weight), L1-normalised
using BowVec = std::vector<std::pair<int, float>>;

struct VocNode {
  uint8_t d[kDescBytes];
  int32_t child0 = -1;  // index of the first child (children are contiguous), -1 = leaf
  int32_t n_child = 0;
  int32_t word = -1;    // leaf: word id
};

class Vocab {
public:
  int k = 0, L = 0;
  std::vector<VocNode> nodes;  // nodes[0] = root (no descriptor)
  std::vector<float> idf;      // per word
  int n_words() const { return static_cast<int>(idf.size()); }
  bool empty() const { return nodes.empty(); }
  bool load(const std::string &path, std::string &err);
  bool save(const std::string &path) const;
  int word_of(const uint8_t *d) const;
  // desc: N x 32 CV_8U rows -> TF-IDF, L1 normalised
  void transform(const cv::Mat &desc, BowVec &v) const;
  // hierarchical k-majority (k-means++ seeding, `iters` Lloyd rounds per node); idf from `doc_of` (descriptor -> image id)
  void train(const std::vector<const uint8_t *> &data, const std::vector<int> &doc_of, int k, int L, int iters,
             uint32_t seed);
  std::string sha() const;  // short content hash (logged)
};

// DBoW2 L1 score of two L1-normalised vectors: 1 - 0.5 * |a - b|_1 in [0, 1]
float bow_score(const BowVec &a, const BowVec &b);

// inverted-file database of keyframe BoW vectors
class BowDb {
public:
  void clear() {
    inv_.clear();
    n_ = 0;
  }
  int add(const BowVec &v);  // returns the entry index (0, 1, ...)
  // scores of all entries with index < max_entry sharing >= 1 word, as (entry, score), unsorted
  void query(const BowVec &v, int max_entry, std::vector<std::pair<int, float>> &out) const;
  int size() const { return n_; }

private:
  std::unordered_map<int, std::vector<std::pair<int, float>>> inv_;  // word -> (entry, weight)
  int n_ = 0;
};

}  // namespace ovloop
