#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <limits>

namespace openstm::lib {
template <size_t MAX_SUBSCRIBERS, typename... Args>
class Event {
 public:
  class Subscription {
    friend Event;
    static constexpr size_t RESET = std::numeric_limits<size_t>::max();

    Event* m_pParent{nullptr};
    size_t m_ID{RESET};

    Subscription(Event* pParent, size_t id) : m_pParent(pParent), m_ID(id) {}

   public:
    Subscription() = default;
    ~Subscription() { Reset(); }

    Subscription(const Subscription& other) = delete;
    Subscription(Subscription&& other) noexcept
        : m_pParent(other.m_pParent), m_ID(other.m_ID) {
      other.Reset();
      if (m_pParent && m_ID < m_pParent->m_Subscribers.size()) {
        m_pParent->m_Subscribers[m_ID].Subscriber = this;
      }
    }

    Subscription& operator=(const Subscription& other) = delete;
    Subscription& operator=(Subscription&& other) noexcept {
      Reset();
      m_pParent = other.m_pParent;
      m_ID = other.m_ID;
      other.Reset();
      if (m_pParent && m_ID < m_pParent->m_Subscribers.size()) {
        m_pParent->m_Subscribers[m_ID].Subscriber = this;
      }
      return *this;
    }

    bool IsSubscribed() { return m_pParent; }

    void Reset() {
      if (m_pParent) {
        m_pParent->m_Subscribers[m_ID].Subscriber = nullptr;
        m_pParent = nullptr;
        m_ID = RESET;
      }
    }
  };

 private:
  struct SubscriberData {
    Subscription* Subscriber{nullptr};
    std::function<void(Args...)> Callback;
  };

  friend Subscription;
  std::array<SubscriberData, MAX_SUBSCRIBERS> m_Subscribers{};

 public:
  Event() = default;
  ~Event() {
    for (auto& sub : m_Subscribers) {
      if (sub.Subscriber) {
        sub.Subscriber->m_pParent = nullptr;
      }
    }
  }

  Subscription Subscribe(std::function<void(Args...)> callback) {
    size_t newID = getFreeID();
    if (newID < m_Subscribers.size()) {
      Subscription sub{this, newID};
      m_Subscribers[newID].Subscriber = &sub;
      m_Subscribers[newID].Callback = std::move(callback);
      return sub;
    }
    return {};
  }

  void Invoke(Args... args) {
    for (auto& sub : m_Subscribers) {
      if (sub.Subscriber) {
        sub.Callback(args...);
      }
    }
  }

 private:
  size_t getFreeID() {
    for (size_t i = 0; i < m_Subscribers.size(); ++i) {
      if (!m_Subscribers[i].Subscriber) {
        return i;
      }
    }
    return m_Subscribers.size();
  }
};
}  // namespace openstm::lib
