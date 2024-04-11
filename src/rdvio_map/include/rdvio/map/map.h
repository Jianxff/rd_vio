#pragma once

#include <rdvio/types.h>

namespace rdvio {

class Frame;
class Track;
class MarginalizationFactor;
class VirtualObjectManager;
class Localizer;

class Map {
    friend class Track;
    struct construct_by_map_t;

  public:
    Map();
    virtual ~Map();

    void clear();

    size_t frame_num() const { return frames.size(); }

    Frame *get_frame(size_t index) const { return frames[index].get(); }

    void attach_frame(std::unique_ptr<Frame> frame, size_t position = nil());
    std::unique_ptr<Frame> detach_frame(size_t index);

    void untrack_frame(Frame *frame);
    void erase_frame(size_t index);
    void marginalize_frame(size_t index);

    size_t frame_index_by_id(size_t id) const;

    size_t track_num() const { return tracks.size(); }

    Track *get_track(size_t index) const { return tracks[index].get(); }

    Track *create_track();
    void erase_track(Track *track);

    void prune_tracks(const std::function<bool(const Track *)> &condition);

    Track *get_track_by_id(size_t id) const;

    std::unique_lock<std::mutex> lock() const {
        return std::unique_lock(map_mutex);
    }

    std::unique_ptr<MarginalizationFactor> marginalization_factor;

  private:
    void recycle_track(Track *track);

    std::deque<std::unique_ptr<Frame>> frames;
    std::vector<std::unique_ptr<Track>> tracks;
    std::map<size_t, Track *> track_id_map;
    mutable std::mutex map_mutex;
};

} // namespace rdvio

