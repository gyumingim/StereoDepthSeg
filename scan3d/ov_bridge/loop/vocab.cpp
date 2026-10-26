// ovloop vocab.cpp (2026-10-26): see vocab.h
#include "vocab.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <random>
#include <set>

namespace ovloop {

namespace {
constexpr uint32_t kVocMagic = 0x564C564Fu;  // "OVLV"
constexpr int32_t kVocVersion = 1;

uint64_t fnv1a(const void *p, size_t n, uint64_t h) {
  const uint8_t *b = static_cast<const uint8_t *>(p);
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 1099511628211ull;
  }
  return h;
}
}  // namespace

bool Vocab::load(const std::string &path, std::string &err) {
  FILE *f = std::fopen(path.c_str(), "rb");
  if (!f) {
    err = "cannot open " + path;
    return false;
  }
  uint32_t magic = 0;
  int32_t ver = 0, kk = 0, ll = 0, nn = 0, nw = 0;
  bool ok = std::fread(&magic, 4, 1, f) == 1 && std::fread(&ver, 4, 1, f) == 1 && std::fread(&kk, 4, 1, f) == 1 &&
            std::fread(&ll, 4, 1, f) == 1 && std::fread(&nn, 4, 1, f) == 1 && std::fread(&nw, 4, 1, f) == 1;
  if (!ok || magic != kVocMagic || ver != kVocVersion || nn <= 1 || nw <= 0 || nn > 10000000 || nw > nn) {
    std::fclose(f);
    err = "bad vocabulary header " + path;
    return false;
  }
  nodes.assign(nn, VocNode());
  idf.assign(nw, 0.0f);
  for (auto &n : nodes)
    ok = ok && std::fread(n.d, kDescBytes, 1, f) == 1 && std::fread(&n.child0, 4, 1, f) == 1 &&
         std::fread(&n.n_child, 4, 1, f) == 1 && std::fread(&n.word, 4, 1, f) == 1;
  ok = ok && std::fread(idf.data(), sizeof(float), nw, f) == static_cast<size_t>(nw);
  std::fclose(f);
  if (!ok) {
    nodes.clear();
    idf.clear();
    err = "truncated vocabulary " + path;
    return false;
  }
  for (const auto &n : nodes)  // structural check (a corrupt file must not crash the bridge)
    if ((n.child0 >= 0 && (n.child0 + n.n_child > nn || n.n_child <= 0)) || (n.child0 < 0 && (n.word < 0 || n.word >= nw))) {
      nodes.clear();
      idf.clear();
      err = "corrupt vocabulary tree " + path;
      return false;
    }
  k = kk;
  L = ll;
  return true;
}

bool Vocab::save(const std::string &path) const {
  FILE *f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  int32_t nn = static_cast<int32_t>(nodes.size()), nw = static_cast<int32_t>(idf.size());
  std::fwrite(&kVocMagic, 4, 1, f);
  std::fwrite(&kVocVersion, 4, 1, f);
  std::fwrite(&k, 4, 1, f);
  std::fwrite(&L, 4, 1, f);
  std::fwrite(&nn, 4, 1, f);
  std::fwrite(&nw, 4, 1, f);
  for (const auto &n : nodes) {
    std::fwrite(n.d, kDescBytes, 1, f);
    std::fwrite(&n.child0, 4, 1, f);
    std::fwrite(&n.n_child, 4, 1, f);
    std::fwrite(&n.word, 4, 1, f);
  }
  std::fwrite(idf.data(), sizeof(float), nw, f);
  return std::fclose(f) == 0;
}

std::string Vocab::sha() const {
  uint64_t h = 1469598103934665603ull;
  for (const auto &n : nodes) {
    h = fnv1a(n.d, kDescBytes, h);
    h = fnv1a(&n.child0, 4, h);
    h = fnv1a(&n.n_child, 4, h);
    h = fnv1a(&n.word, 4, h);
  }
  h = fnv1a(idf.data(), idf.size() * sizeof(float), h);
  char b[32];
  std::snprintf(b, sizeof(b), "%016llx", static_cast<unsigned long long>(h));
  return b;
}

int Vocab::word_of(const uint8_t *d) const {
  int cur = 0;
  while (nodes[cur].child0 >= 0) {
    const int c0 = nodes[cur].child0, nc = nodes[cur].n_child;
    int best = c0, bd = 1 << 30;
    for (int c = c0; c < c0 + nc; c++) {
      int dd = hamming32(d, nodes[c].d);
      if (dd < bd) {
        bd = dd;
        best = c;
      }
    }
    cur = best;
  }
  return nodes[cur].word;
}

void Vocab::transform(const cv::Mat &desc, BowVec &v) const {
  v.clear();
  if (desc.empty() || nodes.empty()) return;
  std::map<int, float> acc;
  for (int i = 0; i < desc.rows; i++) acc[word_of(desc.ptr<uint8_t>(i))] += 1.0f;
  double s = 0.0;
  for (auto &kv : acc) {
    kv.second *= idf[kv.first];
    s += std::fabs(kv.second);
  }
  if (s <= 0.0) return;
  v.reserve(acc.size());
  for (const auto &kv : acc)
    if (kv.second > 0.0f) v.emplace_back(kv.first, static_cast<float>(kv.second / s));
}

void Vocab::train(const std::vector<const uint8_t *> &data, const std::vector<int> &doc_of, int kk, int ll, int iters,
                  uint32_t seed) {
  k = kk;
  L = ll;
  nodes.assign(1, VocNode());
  std::memset(nodes[0].d, 0, kDescBytes);
  std::mt19937 rng(seed);
  int n_words = 0;
  std::vector<int> word_of_desc(data.size(), -1);
  // recursive k-majority clustering of `ids` below node `ni` (level = depth of ni)
  std::function<void(int, std::vector<int> &, int)> build = [&](int ni, std::vector<int> &ids, int level) {
    if (level >= L || static_cast<int>(ids.size()) <= 1) {  // leaf = word
      nodes[ni].word = n_words++;
      for (int i : ids) word_of_desc[i] = nodes[ni].word;
      return;
    }
    const int kc = std::min<int>(k, static_cast<int>(ids.size()));
    // k-means++ seeding (Hamming distance)
    std::vector<std::array<uint8_t, kDescBytes>> cen;
    cen.reserve(kc);
    {
      std::uniform_int_distribution<size_t> u(0, ids.size() - 1);
      std::array<uint8_t, kDescBytes> c0;
      std::memcpy(c0.data(), data[ids[u(rng)]], kDescBytes);
      cen.push_back(c0);
      std::vector<double> dmin(ids.size(), 1e30);
      while (static_cast<int>(cen.size()) < kc) {
        double tot = 0.0;
        for (size_t j = 0; j < ids.size(); j++) {
          double d = hamming32(data[ids[j]], cen.back().data());
          dmin[j] = std::min(dmin[j], d * d);
          tot += dmin[j];
        }
        if (tot <= 0.0) break;  // all identical
        std::uniform_real_distribution<double> ur(0.0, tot);
        double r = ur(rng), acc = 0.0;
        size_t pick = ids.size() - 1;
        for (size_t j = 0; j < ids.size(); j++) {
          acc += dmin[j];
          if (acc >= r) {
            pick = j;
            break;
          }
        }
        std::array<uint8_t, kDescBytes> c;
        std::memcpy(c.data(), data[ids[pick]], kDescBytes);
        cen.push_back(c);
      }
    }
    const int nc = static_cast<int>(cen.size());
    std::vector<int> asg(ids.size(), -1);
    for (int it = 0; it < iters; it++) {
      bool changed = false;
      for (size_t j = 0; j < ids.size(); j++) {
        int best = 0, bd = 1 << 30;
        for (int c = 0; c < nc; c++) {
          int d = hamming32(data[ids[j]], cen[c].data());
          if (d < bd) {
            bd = d;
            best = c;
          }
        }
        if (asg[j] != best) {
          asg[j] = best;
          changed = true;
        }
      }
      if (!changed && it > 0) break;
      // majority vote per bit
      std::vector<std::array<int, kDescBytes * 8>> cnt(nc);
      std::vector<int> num(nc, 0);
      for (auto &a : cnt) a.fill(0);
      for (size_t j = 0; j < ids.size(); j++) {
        const uint8_t *d = data[ids[j]];
        auto &a = cnt[asg[j]];
        num[asg[j]]++;
        for (int b = 0; b < kDescBytes * 8; b++) a[b] += (d[b >> 3] >> (7 - (b & 7))) & 1;
      }
      for (int c = 0; c < nc; c++) {
        if (num[c] == 0) continue;  // keep the old centre
        std::memset(cen[c].data(), 0, kDescBytes);
        for (int b = 0; b < kDescBytes * 8; b++)
          if (2 * cnt[c][b] > num[c]) cen[c][b >> 3] |= static_cast<uint8_t>(1u << (7 - (b & 7)));
      }
    }
    std::vector<std::vector<int>> mem(nc);
    for (size_t j = 0; j < ids.size(); j++) mem[asg[j]].push_back(ids[j]);
    // drop empty clusters
    std::vector<int> keep;
    for (int c = 0; c < nc; c++)
      if (!mem[c].empty()) keep.push_back(c);
    const int c0 = static_cast<int>(nodes.size());
    nodes.resize(nodes.size() + keep.size());
    nodes[ni].child0 = c0;
    nodes[ni].n_child = static_cast<int>(keep.size());
    for (size_t q = 0; q < keep.size(); q++) std::memcpy(nodes[c0 + q].d, cen[keep[q]].data(), kDescBytes);
    for (size_t q = 0; q < keep.size(); q++) {
      std::vector<int> m;
      m.swap(mem[keep[q]]);
      build(c0 + static_cast<int>(q), m, level + 1);
    }
  };
  std::vector<int> all(data.size());
  for (size_t i = 0; i < data.size(); i++) all[i] = static_cast<int>(i);
  build(0, all, 0);
  // idf = log(N_docs / docs containing the word)
  std::set<int> docs(doc_of.begin(), doc_of.end());
  const double N = std::max<size_t>(1, docs.size());
  std::vector<std::set<int>> wd(n_words);
  for (size_t i = 0; i < data.size(); i++) wd[word_of_desc[i]].insert(doc_of[i]);
  idf.assign(n_words, 0.0f);
  for (int w = 0; w < n_words; w++) idf[w] = static_cast<float>(std::log(N / std::max<size_t>(1, wd[w].size())));
}

float bow_score(const BowVec &a, const BowVec &b) {
  double s = 0.0;
  size_t i = 0, j = 0;
  while (i < a.size() && j < b.size()) {
    if (a[i].first == b[j].first) {
      const double x = a[i].second, y = b[j].second;
      s += std::fabs(x) + std::fabs(y) - std::fabs(x - y);
      i++;
      j++;
    } else if (a[i].first < b[j].first)
      i++;
    else
      j++;
  }
  return static_cast<float>(0.5 * s);
}

int BowDb::add(const BowVec &v) {
  const int e = n_++;
  for (const auto &wv : v) inv_[wv.first].emplace_back(e, wv.second);
  return e;
}

void BowDb::query(const BowVec &v, int max_entry, std::vector<std::pair<int, float>> &out) const {
  out.clear();
  std::unordered_map<int, double> acc;
  for (const auto &wv : v) {
    auto it = inv_.find(wv.first);
    if (it == inv_.end()) continue;
    const double x = wv.second;
    for (const auto &ew : it->second) {
      if (ew.first >= max_entry) break;  // entries are appended in order
      const double y = ew.second;
      acc[ew.first] += x + y - std::fabs(x - y);
    }
  }
  out.reserve(acc.size());
  for (const auto &kv : acc) out.emplace_back(kv.first, static_cast<float>(0.5 * kv.second));
}

}  // namespace ovloop
