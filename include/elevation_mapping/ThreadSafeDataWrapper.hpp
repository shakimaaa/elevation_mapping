#pragma once

#include <mutex>
#include <thread>

namespace elevation_mapping {

/**
 * 一个小包装器，用于序列化对象的读/写访问以在多线程场景中使用它。
 * @tparam Data the dataclass to wrapp.
 */
template <typename Data>
class ThreadSafeDataWrapper {
 public:
  ThreadSafeDataWrapper() = default;

  ThreadSafeDataWrapper(const ThreadSafeDataWrapper<Data>& other) : data_(other.getData()) {}

  //! @param data to write. For partial updates of the data use getDataToWrite().
  void setData(Data data) {
    std::lock_guard<std::mutex> _{dataMutex_};
    data_ = data;
  }

  //!  @return a copy of the data.
  Data getData() const {
    std::lock_guard<std::mutex> _{dataMutex_};
    return data_;
  }

  //! @return a writable reference to write the data and a guard to prevent concurrent data access while doing so.
  std::pair<Data&, std::unique_lock<std::mutex>> getDataToWrite() {
    std::unique_lock<std::mutex> dataWriteGuard{dataMutex_};
    return {data_, std::move(dataWriteGuard)};
  }

 private:
  //! The data to wrap and guard its access.
  Data data_;
  //! Mutex to hold whenever reading/ writing data_.
  mutable std::mutex dataMutex_;
};

}  // namespace elevation_mapping