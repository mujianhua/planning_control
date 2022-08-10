#pragma once

#include <unordered_map>
#include <vector>
#include <boost/thread/shared_mutex.hpp>

namespace planning {

template <typename I, typename T> class IndexedList {
  public:
    T *Add(const I id, const T &object) {
        auto obs = Find(id);
        if (obs) {
            *obs = object;
            return obs;
        } else {
            object_dict_.insert({id, object});
            auto *ptr = &object_dict_.at(id);
            object_list_.push_back(ptr);
            return ptr;
        }
    }

    T *Find(const I id) {
        auto iter = object_dict_.find(id);
        if (iter != object_dict_.end()) {
            return &iter->second;
        } else {
            return nullptr;
        }
    }

    const size_t Size() { return object_list_.size(); }

    const std::vector<const T *> &Items() const { return object_list_; }

    const std::unordered_map<I, T> &Dict() const { return object_dict_; }

    /**
     * @brief Copy the container with objects.
     */
    IndexedList &operator=(const IndexedList &other) {
        this->object_list_.clear();
        this->object_dict_.clear();
        for (const auto &item : other.Dict()) {
            Add(item.first, item.second);
        }
        return *this;
    }

    void ClearAll() {
        object_list_.clear();
        object_dict_.clear();
    }

    void Clear(const I id) {
        object_dict_.erase(object_dict_.find(id));
        object_list_.clear();
        for (const auto &iter : object_dict_) {
            object_list_.push_back(&iter.second);
        }
    }

  private:
    std::vector<const T *> object_list_;
    std::unordered_map<I, T> object_dict_;
};

template <typename I, typename T>
class ThreadSafeIndexedList : public IndexedList<I, T> {
  public:
    T *Add(const I id, const T &object) {
        boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
        return IndexedList<I, T>::Add(id, object);
    }

    T *Find(const I id) {
        boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
        return IndexedList<I, T>::Find(id);
    }

    std::vector<const T *> Items() const {
        boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
        return IndexedList<I, T>::Items();
    }

  private:
    mutable boost::shared_mutex mutex_;
};

} // namespace planning
