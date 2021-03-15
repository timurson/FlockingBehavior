#ifndef CS561_FLOCKER_H
#define CS561_FLOCKER_H

#include <vector>
#include <unordered_map>
#include <random>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

# define TWO_PI 6.28318530717958647692

enum class DistanceType {
    LINEAR, INVERSE_LINEAR, QUADRATIC, INVERSE_QUADRATIC
};

glm::vec3 clampLength(glm::vec3 v, float length) {
    float len = glm::length(v);
    if (len > length) {
        return glm::normalize(v) * length;
    }
    return v;
}

inline double random_double() {
    static std::uniform_real_distribution<double> distribution(0.0, 1.0);
    static std::mt19937 generator;
    return distribution(generator);
}

inline double random_double(double min, double max) {
    // Returns a random real in [min,max).
    return min + (max - min) * random_double();
}

glm::vec3 getRandomUniform(std::mt19937& engine) {
    std::uniform_real_distribution<float> thetaRange(0.0f, TWO_PI);
    std::uniform_real_distribution<float> oneRange(0, 1);
    float theta = thetaRange(engine);
    float r = sqrt(oneRange(engine));
    float z = sqrt(1.0f - r * r) * (oneRange(engine) > 0.5f ? -1.0f : 1.0f);
    return glm::vec3(r * cos(theta), r * sin(theta), z);
}

struct Boid {
    glm::vec3 position,
              velocity,
              acceleration,
              motion_normal = glm::vec3(0, 0, 1); // normal to plane of motion
    float size = 1.0f;
    bool avoidance = false;
    explicit Boid(glm::vec3 pos, glm::vec3 vel) : position(pos), velocity(vel), acceleration(glm::vec3(0)) {}
};



struct NearbyBoid {
    Boid* boid;
    glm::vec3 direction;
    float distance;
};

struct Vec3Hasher {
    typedef std::size_t result_type;

    result_type operator()(glm::vec3 const &v) const {
        result_type const h1(std::hash<float>()(v.x));
        result_type const h2(std::hash<float>()(v.y));
        result_type const h3(std::hash<float>()(v.z));
        return (h1 * 31 + h2) * 31 + h3;
    }
};

class Flocker {
public:
    // Perception refers to the vision of each boid.  Only boids within this distance influence each other.
    float PerceptionRadius = 30;
    
    // How much boids repel each other
    float SeparationWeight = 3.5;
    DistanceType SeparationType = DistanceType::INVERSE_QUADRATIC;

    float AlignmentWeight = 0.1;
    float CohesionWeight = 1;

    float SteeringWeight = 4.0;
    std::vector<glm::vec3> SteeringTargets;
    DistanceType SteeringTargetType = DistanceType::LINEAR;

    // field of view of our agent in degrees
    float FOVAngleDeg = 20;
    float MaxAcceleration = 5;
    float MaxVelocity = 5;

    // sphere to avoid collision with
    float CollisionRadius = 1.0f;
    glm::vec3 CollisionCenter;

    Flocker() {}

    explicit Flocker(std::vector<Boid> *entities) : boids(entities) {
        std::random_device rd;
        eng = std::mt19937(rd());
    }

    void update(float dt) {
        const float RESPONSE = 0.1f;

        FOVAngleDegCompareValue = cosf(TWO_PI * FOVAngleDeg / 360.0f);
        updateAcceleration();

        for (auto &boid : *boids) {
            glm::vec3 target = avoidanceDirection(boid);
            if (glm::length(target) > 0.001f && boid.avoidance) {
                boid.velocity += dt * (glm::length(boid.velocity) * target - boid.velocity) / RESPONSE;
            }
            boid.velocity = clampLength(boid.velocity + boid.acceleration * dt, MaxVelocity);
            boid.position += boid.velocity * dt;
            if (glm::length2(boid.position - CollisionCenter) < CollisionRadius * CollisionRadius) {
                boid.velocity += 0.1f * boid.position - CollisionCenter;
            }
        }
    }

    void updateAcceleration() {
        if (PerceptionRadius == 0) {
            PerceptionRadius = 1;
        }
        buildVoxelCache();
        for (auto& boid : *boids) {
            updateBoid(boid);
        }
    }

    void buildVoxelCache() {
        voxelCache.clear();
        voxelCache.reserve(boids->size());
        for (auto &b : *boids) {
            voxelCache[getVoxelForBoid(b)].push_back(&b);
        }
    }

    glm::vec3 getVoxelForBoid(const Boid &b) const {
        float radius = std::abs(PerceptionRadius);
        const glm::vec3 &p = b.position;
        glm::vec3 voxelPos;
        voxelPos.x = static_cast<int>(p.x / radius);
        voxelPos.y = static_cast<int>(p.y / radius);
        voxelPos.z = static_cast<int>(p.z / radius);
        return voxelPos;
    }


private:
    std::vector<Boid> *boids;
    std::unordered_map<glm::vec3, std::vector<Boid*>, Vec3Hasher> voxelCache;
    std::mt19937 eng;
    float FOVAngleDegCompareValue = 0; // = cos(PI2 * FOVAngleDeg / 360)

    struct NearbyBoidsInformation
    {
        glm::vec3 separationSum;
        glm::vec3 headingSum;
        glm::vec3 positionSum;
        int count;
    };

    void updateBoid(Boid& b) {
        glm::vec3 separationSum(0);
        glm::vec3 headingSum(0);
        glm::vec3 positionSum(0);
        glm::vec3 pos = b.position;

        auto nearby = getNearbyBoids(b);

        for (NearbyBoid& closeBoid : nearby) {
            if (closeBoid.distance == 0) {
                separationSum += getRandomUniform(eng) * 1000.0f;
            }
            else {
                float separationFactor = transformDistance(closeBoid.distance, SeparationType);
                separationSum += -closeBoid.direction * separationFactor;  // moving away from neighbor boid

            }
            headingSum += closeBoid.boid->velocity;
            positionSum += closeBoid.boid->position;
        }

        glm::vec3 steeringTarget = b.position;
        float targetDistance = -1;
        for (auto &target : SteeringTargets) {
            float distance = transformDistance(glm::length(b.position - target), SteeringTargetType);
            if (targetDistance < 0 || distance < targetDistance) {
                steeringTarget = target;
                targetDistance = distance;

            }
        }

        // Separation: steer to avoid crowding local agents
        glm::vec3 separation = nearby.size() > 0 ? separationSum / (float)nearby.size() : separationSum;

        // Alignment: steer towards the average heading of local agents
        glm::vec3 alignment = nearby.size() > 0 ? headingSum / (float)nearby.size() : headingSum;

        // Cohesion: steer to move toward the average position of local agents
        glm::vec3 avgPosition = nearby.size() > 0 ? positionSum / (float)nearby.size() : b.position;
        glm::vec3 cohesion = avgPosition - b.position;

        // Steering: steer towards the nearest world target location (like a moth to the light)
        glm::vec3 steering(0);
        // avoid division by zero
        if (!glm::any(glm::equal(steeringTarget, b.position))) {
            steering = glm::normalize(steeringTarget - b.position) * targetDistance;
        }

        // calculate boid acceleration using operator splitting
        glm::vec3 acceleration(0);
        acceleration += separation * SeparationWeight;  // w1 * a1
        acceleration += alignment * AlignmentWeight;    // w2 * a2
        acceleration += cohesion * CohesionWeight;      // w3 * a3
        acceleration += steering * SteeringWeight;      // w4 * a4
        b.acceleration = clampLength(acceleration, MaxAcceleration);
    }

    std::vector<NearbyBoid> getNearbyBoids(const Boid& b) const {
        std::vector<NearbyBoid> result;
        result.reserve(boids->size());

        glm::vec3 voxelPos = getVoxelForBoid(b);
        voxelPos.x -= 1;
        voxelPos.y -= 1;
        voxelPos.z -= 1;
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                for (int z = 0; z < 3; z++) {
                    checkVoxelForBoids(b, result, voxelPos);
                    voxelPos.z++;
                }
                voxelPos.z -= 3;
                voxelPos.y++;
            }
            voxelPos.y -= 3;
            voxelPos.x++;
        }
        return result;

    }

    void checkVoxelForBoids(const Boid &b, std::vector<NearbyBoid> &result, const glm::vec3& voxelPos) const {
        auto iter = voxelCache.find(voxelPos);
        if (iter != voxelCache.end()) {
            for (Boid *test : iter->second) {
                const glm::vec3 &p1 = b.position;
                const glm::vec3 &p2 = test->position;
                glm::vec3 vec = p2 - p1;
                float distance = glm::length(vec);

                float compareValue = 0.0f;
                float l1 = glm::length(vec);
                float l2 = glm::length(b.velocity);
                if (l1 != 0 && l2 != 0) {
                    compareValue = glm::dot(-b.velocity, vec) / (l1 * l2);
                }

                if ((&b) != test && distance <= PerceptionRadius && (FOVAngleDegCompareValue > compareValue || glm::length(b.velocity) == 0)) {
                    NearbyBoid nb;
                    nb.boid = test;
                    nb.distance = distance;
                    nb.direction = vec;
                    result.push_back(nb);
                }
            }
        }
    }

    glm::vec3 avoidanceDirection(Boid& boid) const {
        
        float R = CollisionRadius + 0.5f; // adding a little padding to collision radius
        float r2 = R * R;
        float a = glm::length2(boid.velocity);
        float b = glm::dot(2.f * boid.velocity, boid.position - CollisionCenter);
        float c = glm::length2(boid.position - CollisionCenter) - r2;
        float delta = b*b - 4*a*c;

        // seeing check
        if (!(delta >= 0 && b + sqrt(delta) <= 0)) {
            boid.avoidance = false;
            return boid.velocity;
        }
            
        // range check
        if (-(b + sqrt(delta)) * glm::length(boid.velocity) / (2.f*a) >= R) {
            boid.avoidance = false;
            return boid.velocity;
        }
        boid.avoidance = true;

        float s = sqrt(r2 - pow(glm::dot(boid.motion_normal, CollisionCenter - boid.position), 2));
        glm::vec3 C = CollisionCenter - glm::dot(boid.motion_normal, CollisionCenter - boid.position) * boid.motion_normal;

        float sign = glm::dot(boid.motion_normal, glm::cross(C - boid.position, boid.velocity)) > 0 ? 1.0 : -1.0;

        float CPlength = glm::length(C - boid.position);
        float sin_theta = s / CPlength;
        float cos_theta = sqrt(1.f - sin_theta * sin_theta);

        glm::vec3 u = cos_theta * (C - boid.position) / CPlength
            + sign * sin_theta * glm::cross(boid.motion_normal, C - boid.position) / CPlength;
        
        return u;
    }



    float transformDistance(float distance, DistanceType type) {
        if (type == DistanceType::LINEAR) {
            return distance;
        }
        else if (type == DistanceType::INVERSE_LINEAR) {
            return distance == 0 ? 0 : 1 / distance;
        }
        else if (type == DistanceType::QUADRATIC) {
            return std::pow(distance, 2);
        }
        else if (type == DistanceType::INVERSE_QUADRATIC) {
            float quad = std::pow(distance, 2);
            return quad == 0 ? 0 : 1 / quad;
        }
        else {
            return distance; // this shouldn't really happen
        }
    }

};


#endif
