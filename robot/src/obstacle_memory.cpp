#include "obstacle_memory.hpp"

#include <algorithm>
#include <cmath>

#include "log.hpp"

namespace fms {

void ObstacleMemory::update(double dt, double x, double y, double sensor_range,
                            const std::vector<DynamicObstacle>& detected, const std::string& log_id) {
    if (config.base_ttl <= 0) { items_.clear(); return; }

    std::vector<std::string> seen;
    for (auto& o : detected) {
        seen.push_back(o.obstacle_id);
        auto it = items_.find(o.obstacle_id);
        if (it == items_.end()) {
            Item item{o, o.x, o.y, 0.0, config.base_ttl};
            item.moving = o.source == ObstacleSource::SIM || o.target.has_value();
            items_.emplace(o.obstacle_id, std::move(item));
            continue;
        }
        Item& m = it->second;
        if (m.expired && std::hypot(o.x - m.obs.x, o.y - m.obs.y) <= config.same_place) {
            m.ttl = std::min(m.ttl * 2.0, config.max_ttl);  // 같은 자리에 아직 있다 -> 더 오래 피한다
            log::info("[%s] obstacle seen again: %s ttl -> %.0fs", log_id.c_str(), o.obstacle_id.c_str(), m.ttl);
        } else if (m.expired) {
            m.ttl = config.base_ttl;
            m.first_x = o.x; m.first_y = o.y;
        }
        if (std::hypot(o.x - m.first_x, o.y - m.first_y) > config.moving_eps) m.moving = true;
        m.obs = o;
        m.visible = true;
        m.unseen = 0.0;
        m.expired = false;
    }

    for (auto it = items_.begin(); it != items_.end();) {
        Item& m = it->second;
        if (std::find(seen.begin(), seen.end(), it->first) != seen.end()) { ++it; continue; }
        const bool was_visible = m.visible;
        m.visible = false;
        if (m.moving) { it = items_.erase(it); continue; }  // 움직이는 건 잊는다 (유령 사각형 방지)
        if (m.obs.distance_to(x, y) <= sensor_range) {      // 범위 안인데 안 보인다 = 치워졌다
            log::info("[%s] obstacle gone (checked in range): %s", log_id.c_str(), it->first.c_str());
            it = items_.erase(it);
            continue;
        }
        if (was_visible)
            log::info("[%s] obstacle remembered: %s (%.2f, %.2f) ttl=%.0fs", log_id.c_str(), it->first.c_str(), m.obs.x, m.obs.y, m.ttl);
        m.unseen += dt;
        if (!m.expired && m.unseen >= m.ttl) {
            m.expired = true;
            log::info("[%s] obstacle forgotten: %s", log_id.c_str(), it->first.c_str());
        }
        if (m.unseen >= 2.0 * m.ttl + 60.0) { it = items_.erase(it); continue; }
        ++it;
    }
}

std::vector<DynamicObstacle> ObstacleMemory::known(const std::vector<DynamicObstacle>& detected) const {
    std::vector<DynamicObstacle> out = detected;
    for (auto& [id, m] : items_) {
        if (m.visible || m.expired) continue;
        out.push_back(m.obs);
    }
    return out;
}

size_t ObstacleMemory::remembered_count() const {
    size_t n = 0;
    for (auto& [id, m] : items_) if (!m.visible && !m.expired) ++n;
    return n;
}

}  // namespace fms
