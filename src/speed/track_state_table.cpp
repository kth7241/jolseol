#include "speed/track_state_table.hpp"

namespace traffic {

TrackState& TrackStateTable::get(int track_id) {
  return table_[track_id];
}

bool TrackStateTable::has(int track_id) const {
  return table_.find(track_id) != table_.end();
}

void TrackStateTable::erase(int track_id) {
  table_.erase(track_id);
}

void TrackStateTable::purge(int64_t now_ns, int64_t timeout_ns) {
  for (auto it = table_.begin(); it != table_.end(); ) {
    const auto& st = it->second;
    if (st.initialized && (now_ns - st.prev_ts_ns) > timeout_ns) {
      it = table_.erase(it);
    } else {
      ++it;
    }
  }
}

} // namespace traffic
