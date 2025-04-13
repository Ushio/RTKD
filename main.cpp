#include "pr.hpp"
#include <iostream>
#include <memory>

template <class T>
inline glm::vec3 toGLM(T x)
{
    return { x[0], x[1], x[2] };
}

namespace rtkd
{
    template <class T>
    inline T ss_max(T x, T y)
    {
        return (x < y) ? y : x;
    }

    template <class T>
    inline T ss_min(T x, T y)
    {
        return (y < x) ? y : x;
    }
    template <class T>
    inline T ss_clamp(T x, T lower, T upper)
    {
        return ss_min(ss_max(x, lower), upper);
    }

    struct Vec2
    {
        float xs[2];
        float& operator[](int i) { return xs[i]; }
        const float& operator[](int i) const { return xs[i]; }
    };
    struct Vec3
    {
        float xs[3];
        float& operator[](int i) { return xs[i]; }
        const float& operator[](int i) const { return xs[i]; }
    };

    inline Vec3 operator-(Vec3 a, Vec3 b)
    {
        Vec3 r;
        for (int i = 0; i < 3; i++)
        {
            r[i] = a[i] - b[i];
        }
        return r;
    }
    inline Vec3 operator+(Vec3 a, Vec3 b)
    {
        Vec3 r;
        for (int i = 0; i < 3; i++)
        {
            r[i] = a[i] + b[i];
        }
        return r;
    }
    inline Vec3 operator*(Vec3 a, float s)
    {
        Vec3 r;
        for (int i = 0; i < 3; i++)
        {
            r[i] = a[i] * s;
        }
        return r;
    }
    inline Vec3 operator*(Vec3 a, Vec3 b)
    {
        Vec3 r;
        for (int i = 0; i < 3; i++)
        {
            r[i] = a[i] * b[i];
        }
        return r;
    }

    struct Triangle
    {
        Vec3 vs[3];
        Vec3 ng;
    };

    struct AABB
    {
        Vec3 lower;
        Vec3 upper;

        void setEmpty()
        {
            for (int i = 0; i < 3; i++)
            {
                lower[i] = +FLT_MAX;
                upper[i] = -FLT_MAX;
            }
        }
        void extend(const Vec3& p)
        {
            for (int i = 0; i < 3; i++)
            {
                lower[i] = ss_min(lower[i], p[i]);
                upper[i] = ss_max(upper[i], p[i]);
            }
        }
        void growEps( int nEps )
        {
            for (int i = 0; i < 3; i++)
            {
                lower[i] -= ss_max(fabsf(lower[i] * FLT_EPSILON), FLT_MIN) * nEps;
                upper[i] += ss_max(fabsf(upper[i] * FLT_EPSILON), FLT_MIN) * nEps;
            }
        }
        void extend(const AABB& b)
        {
            for (int i = 0; i < 3; i++)
            {
                lower[i] = ss_min(lower[i], b.lower[i]);
                upper[i] = ss_max(upper[i], b.upper[i]);
            }
        }
        float surface_area() const
        {
            Vec3 size = upper - lower;
            return (size[0] * size[1] + size[1] * size[2] + size[2] * size[0]) * 2.0f;
        }
        //float volume() const
        //{
        //    Vec3 size = upper - lower;
        //    return size[0] * size[1] * size[2];
        //}
        bool hasVolume()
        {
            for (int i = 0; i < 3; i++)
            {
                if (upper[i] <= lower[i])
                {
                    return false;
                }
            }
            return true;
        }
        bool isIntersect(const AABB& rhs) const
        {
            for (int i = 0; i < 3; i++)
            {
                if (upper[i] < rhs.lower[i] || rhs.upper[i] < lower[i])
                {
                    return false;
                }
            }
            return true;
        }
        AABB intersect(const AABB& rhs) const
        {
            if (isIntersect(rhs) == false)
            {
                AABB e;
                e.setEmpty();
                return e;
            }
            AABB I;
            for (int i = 0; i < 3; i++)
            {
                I.lower[i] = ss_max(lower[i], rhs.lower[i]);
                I.upper[i] = ss_min(upper[i], rhs.upper[i]);
            }
            return I;
        }
        bool isEmpty()
        {
            for (int i = 0; i < 3; i++)
            {
                if (upper[i] <= lower[i])
                {
                    return true;
                }
            }
            return false;
        }
    };

    struct KDTask
    {
        int beg;
        int end;
        AABB aabb;
        uint64_t slot_ptr;
    };
    struct KDElement
    {
        int triangleIndex;
        AABB aabb;
    };

    //inline int bucket( float x, float lower, float upper, int nBins )
    //{
    //    int index = (int)( (x - lower) * nBins / (upper - lower) );
    //    return ss_min(ss_max(0, index), nBins - 1);
    //}

    // i=0: lower, i=nBins: upper
    inline float border(int i, float lower, float upper, int nBins)
    {
        return lower + (upper - lower) / nBins * i;
    }

    // strictly border-based binning
    inline int bucket(float x, float lower, float upper, int nBins)
    {
        int beg = 0;
        int end = nBins;

        while( beg + 1 < end )
        {
            int mid = (beg + end) / 2;
            float b = border(mid, lower, upper, nBins);
            if (x < b)
            {
                end = mid;
            }
            else
            {
                beg = mid;
            }
        }
        return beg;
    }

    inline void divide_clip(AABB* L, AABB *R, const AABB& baseAABB, Vec3 a, Vec3 b, Vec3 c, float boundary, int axis, int nEps )
    {
        L->setEmpty();
        R->setEmpty();
        Vec3 vs[] = { a, b, c };
        for (int i = 0; i < 3; i++)
        {
            Vec3 ro = vs[i];
            Vec3 rd = vs[(i + 1) % 3] - ro;
            float one_over_rd = ss_clamp(1.0f / rd[axis], -FLT_MAX, FLT_MAX);
            float t = (boundary - ro[axis]) * one_over_rd;
            if( ro[axis] - boundary < 0.0f )
            {
                L->extend(ro);
            }
            else
            {
                R->extend(ro);
            }

            if (-FLT_EPSILON * nEps < t && t < 1.0f + FLT_EPSILON * nEps)
            {
                Vec3 p = ro + rd * ss_clamp(t, 0.0f, 1.0f);
                L->extend(p);
                R->extend(p);
            }
        }

        // conservative bound for axis-aligned element
        L->growEps(nEps);
        R->growEps(nEps);

        *L = baseAABB.intersect(*L);
        *R = baseAABB.intersect(*R);
    }

    static const float COST_TRAVERSE = 1.0f;
    static const float COST_INTERSECT = 8.0f;

    static const uint32_t KD_ALIGNMENT = 16;
    static const uint32_t KD_LEAF_FLAG = 0x80000000;
    static const uint32_t KD_INDEX_MASK = 0x7FFFFFFF;

    static const uint32_t KD_CONTINUE_BIT = 0x80000000;

    struct alignas(KD_ALIGNMENT) KDNode
    {
        int axis;
        float boundary;
        uint32_t childL;
        uint32_t childR;
    };
    struct alignas(KD_ALIGNMENT) KDLeaf
    {
        uint32_t triangleBeg;
        uint32_t triangleEnd;
    };

    static_assert(sizeof(KDNode) == KD_ALIGNMENT, "");
    static_assert(sizeof(KDLeaf) == KD_ALIGNMENT, "");

    inline Vec3 vmin(Vec3 a, Vec3 b)
    {
        Vec3 r;
        for (int axis = 0; axis < 3; axis++)
        {
            r[axis] = ss_min(a[axis], b[axis]);
        }
        return r;
    }
    inline Vec3 vmax(Vec3 a, Vec3 b)
    {
        Vec3 r;
        for (int axis = 0; axis < 3; axis++)
        {
            r[axis] = ss_max(a[axis], b[axis]);
        }
        return r;
    }
    inline float compMin(Vec3 v)
    {
        return ss_min(ss_min(v[0], v[1]), v[2]);
    }
    inline float compMax(Vec3 v)
    {
        return ss_max(ss_max(v[0], v[1]), v[2]);
    }

    inline Vec2 slabs(Vec3 ro, Vec3 one_over_rd, Vec3 lower, Vec3 upper )
    {
        Vec3 t0 = (lower - ro) * one_over_rd;
        Vec3 t1 = (upper - ro) * one_over_rd;

        Vec3 tmin = vmin(t0, t1);
        Vec3 tmax = vmax(t0, t1);
        float region_min = compMax(tmin);
        float region_max = compMin(tmax);

        region_min = fmaxf(region_min, 0.0f);

        return { region_min, region_max };
    }

    inline float dot(Vec3 a, Vec3 b)
    {
        float r = 0.0f;
        for (int axis = 0; axis < 3; axis++)
        {
            r += a[axis] * b[axis];
        }
        return r;
    }

    inline Vec3 cross(Vec3 a, Vec3 b)
    {
        return {
            a[1] * b[2] - b[1] * a[2],
            a[2] * b[0] - b[2] * a[0],
            a[0] * b[1] - b[0] * a[1]
        };
    }

    inline bool intersect_ray_triangle( float* tOut, float* uOut, float* vOut, Vec3* ngOut, float t_min, float t_max, Vec3 ro, Vec3 rd, Vec3 v0, Vec3 v1, Vec3 v2, Vec3 ng)
    {
        float t = dot(v0 - ro, ng) / dot(ng, rd);
        if(t_min <= t && t <= t_max)
        {
            Vec3 e0 = v1 - v0;
            Vec3 e1 = v2 - v1;
            Vec3 e2 = v0 - v2;

            Vec3 p = ro + rd * t;

            float n2TriArea0 = dot(ng, cross(e0, p - v0));  // |n| * 2 * tri_area( p, v0, v1 )
            float n2TriArea1 = dot(ng, cross(e1, p - v1));  // |n| * 2 * tri_area( p, v1, v2 )
            float n2TriArea2 = dot(ng, cross(e2, p - v2));  // |n| * 2 * tri_area( p, v2, v0 )

            if (n2TriArea0 < 0.0f || n2TriArea1 < 0.0f || n2TriArea2 < 0.0f)
            {
                return false;
            }

            float n2TriArea = n2TriArea0 + n2TriArea1 + n2TriArea2;  // |n| * 2 * tri_area( v0, v1, v2 )

            // Barycentric Coordinates
            float bW = n2TriArea0 / n2TriArea;  // tri_area( p, v0, v1 ) / tri_area( v0, v1, v2 )
            float bU = n2TriArea1 / n2TriArea;  // tri_area( p, v1, v2 ) / tri_area( v0, v1, v2 )
            float bV = n2TriArea2 / n2TriArea;  // tri_area( p, v2, v0 ) / tri_area( v0, v1, v2 )

            *tOut = t;
            *uOut = bV;
            *vOut = bW;
            *ngOut = ng;
            return true;
        }

        return false;
    }
    struct KDHit
    {
        float t = FLT_MAX;
        uint32_t triangleIndex = 0xFFFFFFFF;
        Vec3 ng;
    };

    inline void intersect( KDHit *hit, uint32_t nodeIndex, Vec3 ro, Vec3 rd, Vec3 one_over_rd, float t_min, float t_max, uint8_t* kdBuffer, uint32_t* triangleBuffer, Triangle* triangles)
    {
        if (nodeIndex == 0xFFFFFFFF)
        {
            return;
        }
        if ((hit->triangleIndex & KD_CONTINUE_BIT) == 0)
        {
            return;
        }

        // assume root is KDNode
        if (nodeIndex & KD_LEAF_FLAG)
        {
            nodeIndex &= KD_INDEX_MASK;
            KDLeaf* leaf = (KDLeaf*)&kdBuffer[(uint64_t)nodeIndex * KD_ALIGNMENT];
            for (int i = leaf->triangleBeg; i != leaf->triangleEnd; i++)
            {
                uint32_t triangleIndex = triangleBuffer[i];
                Triangle tri = triangles[triangleIndex];

                float t;
                float u, v;
                Vec3 ng;
                if (intersect_ray_triangle(&t, &u, &v, &ng, 0.0f, hit->t, ro, rd, tri.vs[0], tri.vs[1], tri.vs[2], tri.ng))
                {
                    // printf("%.8f %.8f | %.8f -> %.8f\n", t_min, t_max, hit->t, t);
                    hit->t = t;
                    hit->triangleIndex = triangleIndex;
                    hit->ng = ng;

                    if (t < t_min || t_max < t)
                    {
                        hit->triangleIndex |= KD_CONTINUE_BIT;
                    }
                }

                //pr::PrimBegin(pr::PrimitiveMode::Lines, 2);
                //for (int j = 0; j < 3; ++j)
                //{
                //    rtkd::Vec3 v0 = tri.vs[j];
                //    rtkd::Vec3 v1 = tri.vs[(j + 1) % 3];
                //    pr::PrimVertex({ v0.xs[0], v0.xs[1], v0.xs[2] }, { 255, 0, 255 });
                //    pr::PrimVertex({ v1.xs[0], v1.xs[1], v1.xs[2] }, { 255, 0, 255 });
                //}
                //pr::PrimEnd();
            }

            return;
        }

        KDNode* node = (KDNode*)&kdBuffer[(uint64_t)nodeIndex * KD_ALIGNMENT];

        float t = (node->boundary - ro[node->axis]) * one_over_rd[node->axis];

        uint32_t childL = node->childL;
        uint32_t childR = node->childR;
        if (one_over_rd[node->axis] < 0.0f)
        {
            std::swap(childL, childR);
        }

        if (t_max <= t)
        {
            intersect(hit, childL, ro, rd, one_over_rd, t_min, t_max, kdBuffer, triangleBuffer, triangles);
        }
        else if (t <= t_min)
        {
            intersect(hit, childR, ro, rd, one_over_rd, t_min, t_max, kdBuffer, triangleBuffer, triangles);
        }
        else
        {
            intersect(hit, childL, ro, rd, one_over_rd, t_min, t, kdBuffer, triangleBuffer, triangles);
            intersect(hit, childR, ro, rd, one_over_rd, t, t_max, kdBuffer, triangleBuffer, triangles);
        }
    }

    inline void intersect_rs(KDHit* hit, uint32_t rootIndex, Vec3 ro, Vec3 rd, Vec3 one_over_rd, float t_scene_min, float t_scene_max, uint8_t* kdBuffer, uint32_t* triangleBuffer, Triangle* triangles)
    {
        uint32_t nodeIndex = rootIndex;
        float t_min = t_scene_min;
        float t_max = t_scene_max;
        while ( t_min < t_scene_max )
        {
            while ((nodeIndex & KD_LEAF_FLAG) == 0)
            {
                KDNode* node = (KDNode*)&kdBuffer[(uint64_t)nodeIndex * KD_ALIGNMENT];

                float t = (node->boundary - ro[node->axis]) * one_over_rd[node->axis];

                uint32_t childL = node->childL;
                uint32_t childR = node->childR;
                if (one_over_rd[node->axis] < 0.0f)
                {
                    std::swap(childL, childR);
                }

                if (t_max <= t)
                {
                    nodeIndex = childL;
                }
                else if (t <= t_min)
                {
                    nodeIndex = childR;
                }
                else if(childL != 0xFFFFFFFF)
                {
                    nodeIndex = childL;
                    t_max = t;
                }
                else
                {
                    nodeIndex = childR;
                    t_min = t;
                }
            }

            if (nodeIndex != 0xFFFFFFFF)
            {
                nodeIndex &= KD_INDEX_MASK;
                KDLeaf* leaf = (KDLeaf*)&kdBuffer[(uint64_t)nodeIndex * KD_ALIGNMENT];

                for (int i = leaf->triangleBeg; i != leaf->triangleEnd; i++)
                {
                    uint32_t triangleIndex = triangleBuffer[i];
                    Triangle tri = triangles[triangleIndex];

                    float t;
                    float u, v;
                    Vec3 ng;
                    if (intersect_ray_triangle(&t, &u, &v, &ng, t_min, ss_min(hit->t, t_max), ro, rd, tri.vs[0], tri.vs[1], tri.vs[2], tri.ng))
                    {
                        hit->t = t;
                        hit->triangleIndex = triangleIndex;
                        hit->ng = ng;
                    }

                    //pr::PrimBegin(pr::PrimitiveMode::Lines, 2);
                    //for (int j = 0; j < 3; ++j)
                    //{
                    //    rtkd::Vec3 v0 = tri.vs[j];
                    //    rtkd::Vec3 v1 = tri.vs[(j + 1) % 3];
                    //    pr::PrimVertex({ v0.xs[0], v0.xs[1], v0.xs[2] }, { 255, 0, 255 });
                    //    pr::PrimVertex({ v1.xs[0], v1.xs[1], v1.xs[2] }, { 255, 0, 255 });
                    //}
                    //pr::PrimEnd();
                }
                if (hit->triangleIndex != 0xFFFFFFFF)
                {
                    return;
                }
            }

            t_min = t_max;
            t_max = t_scene_max;
            nodeIndex = rootIndex;
        }
    }

    inline void intersect(KDHit *hit, Vec3 ro, Vec3 rd, AABB bounds, uint8_t* kdBuffer, uint32_t* triangleBuffer, Triangle* triangles)
    {
        Vec3 one_over_rd;
        for (int axis = 0; axis < 3; axis++)
        {
            one_over_rd[axis] = ss_clamp(1.0f / rd[axis], -FLT_MAX, FLT_MAX);
        }

        Vec2 range = slabs(ro, one_over_rd, bounds.lower, bounds.upper);
        if (range[1] < range[0] )
        {
            return;
        }

        intersect_rs(hit, 0, ro, rd, one_over_rd, range[0], range[1], kdBuffer, triangleBuffer, triangles);
        //intersect(hit, 0, ro, rd, one_over_rd, range[0], range[1], kdBuffer, triangleBuffer, triangles );

        if( hit->t != FLT_MAX )
        {
            hit->triangleIndex &= ~KD_CONTINUE_BIT;
        }
    }
}


void test()
{
    int nBins = 32;
    pr::PCG rng;
    for (int i = 0; i < 10000; i++)
    {
        float lower = glm::mix( -10.0f, 10.0f, rng.uniformf());
        float upper = lower + 0.0001f + rng.uniformf() * 10.0f;
        for (int j = 0; j < 10000; j++)
        {
            float x = glm::mix( lower, upper, rng.uniformf() ); 
            x = glm::clamp( x, lower, nextafterf(upper, -FLT_MAX));

            int b = rtkd::bucket(x, lower, upper, nBins);

            float L = rtkd::border(b, lower, upper, nBins);
            float R = rtkd::border(b + 1, lower, upper, nBins);

            PR_ASSERT(L <= x);
            PR_ASSERT(x < R);
        }
    }

}

int main() {
    using namespace pr;

    test();

    Config config;
    config.ScreenWidth = 1920;
    config.ScreenHeight = 1080;
    config.SwapInterval = 0;
    Initialize(config);

    Camera3D camera;
    camera.origin = { 0, 3, 3 };
    camera.lookat = { 0, 0, 0 };

    double e = GetElapsedTime();

    SetDataDir(ExecutableDir());
    std::string err;
    std::shared_ptr<FScene> scene = ReadWavefrontObj(GetDataPath("test.obj"), err);
    // std::shared_ptr<FScene> scene = ReadWavefrontObj(GetDataPath("4_tris_flat.obj"), err);
    //std::shared_ptr<FScene> scene = ReadWavefrontObj(GetDataPath("share.obj"), err);
    
    bool showWire = true;

    ITexture* texture = CreateTexture();

    while (pr::NextFrame() == false) {
        if (IsImGuiUsingMouse() == false) {
            UpdateCameraBlenderLike(&camera);
        }

        //ClearBackground(0.1f, 0.1f, 0.1f, 1);
        ClearBackground(texture);

        BeginCamera(camera);

        PushGraphicState();

        DrawGrid(GridAxis::XZ, 1.0f, 10, { 128, 128, 128 });
        DrawXYZAxis(1.0f);

        //static glm::vec3 ro = { 1, 1, 1 };
        // static glm::vec3 ro = { 0.428085, 0.890613, 0.028283 };
        // static glm::vec3 to = { -1, -1, -1 };

        static glm::vec3 ro = { 0.877477, 0.946881, 0.041480 };
        static glm::vec3 to = { 0.370726, -1.000000, -0.195561 };

        glm::vec3 rd = to - ro;
        
        ManipulatePosition(camera, &ro, 0.2f);
        ManipulatePosition(camera, &to, 0.2f);
        DrawLine(ro, ro + rd * 10.0f, { 128, 128, 128 });

        // hmm still some bad aabb..
        static int debug_index = -1;

        scene->visitPolyMesh([&](std::shared_ptr<const FPolyMeshEntity> polymesh) {
            if (polymesh->visible() == false)
            {
                return;
            }
            ColumnView<int32_t> faceCounts(polymesh->faceCounts());
            ColumnView<int32_t> indices(polymesh->faceIndices());
            ColumnView<glm::vec3> positions(polymesh->positions());
            ColumnView<glm::vec3> normals(polymesh->normals());

            auto sheet = polymesh->attributeSpreadsheet(pr::AttributeSpreadsheetType::Points);

            std::vector<rtkd::Triangle> triangles;
            int indexBase = 0;
            for (int i = 0; i < faceCounts.count(); i++)
            {
                int nVerts = faceCounts[i];
                PR_ASSERT(nVerts == 3);
                rtkd::Triangle tri;
                for (int j = 0; j < nVerts; ++j)
                {
                    glm::vec3 p = positions[indices[indexBase + j]];
                    tri.vs[j] = { p.x, p.y, p.z };
                }

                rtkd::Vec3 e0 = tri.vs[1] - tri.vs[0];
                rtkd::Vec3 e1 = tri.vs[2] - tri.vs[1];
                rtkd::Vec3 e2 = tri.vs[0] - tri.vs[2];

                tri.ng = cross(e0, e1);

                triangles.push_back(tri);
                indexBase += nVerts;
            }

            if (showWire)
            {
                pr::PrimBegin(pr::PrimitiveMode::Lines);

                for (auto tri : triangles)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        rtkd::Vec3 v0 = tri.vs[j];
                        rtkd::Vec3 v1 = tri.vs[(j + 1) % 3];
                        pr::PrimVertex({ v0.xs[0], v0.xs[1], v0.xs[2] }, { 255, 255, 255 });
                        pr::PrimVertex({ v1.xs[0], v1.xs[1], v1.xs[2] }, { 255, 255, 255 });
                    }
                }

                pr::PrimEnd();
            }


            static rtkd::AABB sceneBounds;
            static std::vector<uint8_t> kdtreeBuffer;
            static std::vector<uint32_t> triangleBuffer;

            if (kdtreeBuffer.empty())
            {
                sceneBounds.setEmpty();
                kdtreeBuffer.clear();
                triangleBuffer.clear();

                // Build KD Tree
                std::vector<rtkd::KDElement> elementAABBs_inputs(triangles.size() * 8);
                std::vector<rtkd::KDElement> elementAABBs_outputs(triangles.size() * 8);

                for (int i = 0; i < triangles.size(); i++)
                {
                    rtkd::AABB triangleAABB;
                    triangleAABB.setEmpty();
                    for (int j = 0; j < 3; j++)
                    {
                        triangleAABB.extend(triangles[i].vs[j]);
                    }
                    triangleAABB.growEps(64);
                    sceneBounds.extend(triangleAABB);

                    elementAABBs_inputs[i].triangleIndex = i;
                    elementAABBs_inputs[i].aabb = triangleAABB;
                }

                std::vector<rtkd::KDTask> tasks_inputs;
                std::vector<rtkd::KDTask> tasks_outputs;
                tasks_inputs.push_back(
                    {
                        0, (int)triangles.size(),
                        sceneBounds,
                        0xFFFFFFFFFFFFFFFF
                    }
                );


                //int maxDepth = 8 + log2(triangles.size());
                int maxDepth = 32;

                for (int i_task = 0; i_task < maxDepth; i_task++)
                {
                    int output_counter = 0;
                    for (rtkd::KDTask task : tasks_inputs)
                    {
                        int nElement = task.end - task.beg;

                        int best_axis = -1;
                        int best_i_split = -1;
                        int best_nL = 0;
                        int best_nR = 0;
                        rtkd::AABB best_aabbL;
                        rtkd::AABB best_aabbR;
                        float best_cost = nElement * rtkd::COST_INTERSECT;

                        /*
                        i_split
                        0  1  2
                        +--+--+---
                        |  |  |   ..
                        +--+--+---
                
                        bin
                        +--+--+---
                        |0 |1 |2  ..
                        +--+--+---
                        */

                        enum { NBins = 32 };
                        for (int axis = 0; axis < 3; axis++)
                        {
                            if (task.aabb.upper[axis] - task.aabb.lower[axis] < FLT_EPSILON )
                            {
                                continue;
                            }

                            int min_bins[NBins] = {};
                            int max_bins[NBins] = {};

                            for (int i = task.beg; i != task.end; i++)
                            {
                                rtkd::AABB box = elementAABBs_inputs[i].aabb;
                                int index_min = rtkd::bucket(box.lower[axis], task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                                int index_max = rtkd::bucket(box.upper[axis], task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                                min_bins[index_min]++;
                                max_bins[index_max]++;
                            }

                            // inclusive prefix sum
                            int mi_count = 0;
                            int ma_count = 0;
                            for (int i = 0; i < NBins; i++)
                            {
                                mi_count += min_bins[i];
                                min_bins[i] = mi_count;

                                ma_count += max_bins[NBins - i - 1];
                                max_bins[NBins - i - 1] = ma_count;
                            }

                            float p_area = task.aabb.surface_area();
                            for (int i_split = 1; i_split < NBins; i_split++)
                            {

                                float b = rtkd::border(i_split, task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                                rtkd::AABB L = task.aabb;
                                rtkd::AABB R = task.aabb;
                                L.upper[axis] = b;
                                R.lower[axis] = b;

                                int nL = min_bins[i_split - 1];
                                int nR = max_bins[i_split];

                                float cost = rtkd::COST_TRAVERSE +
                                    (L.surface_area() / p_area) * nL * rtkd::COST_INTERSECT +
                                    (R.surface_area() / p_area) * nR * rtkd::COST_INTERSECT;

                                bool healtySplit =
                                    nL == 0 || nR == 0 ||
                                    (nL < nElement && nR < nElement);

                                if( healtySplit )
                                if ( cost < best_cost )
                                {
                                    best_cost = cost;
                                    best_i_split = i_split;
                                    best_axis = axis;
                                    best_nL = nL;
                                    best_nR = nR;
                                    best_aabbL = L;
                                    best_aabbR = R;
                                }
                            }
                        }

                        if (0 < best_i_split)
                        {
                            // this can be tightly fit. although divide_clip uses this boundary but fitting does not affect the clipping.
                            float boundary = rtkd::border(best_i_split, task.aabb.lower[best_axis], task.aabb.upper[best_axis], NBins);
                        
                            int alloc_head = output_counter;
                            output_counter += best_nL + best_nR;

                            PR_ASSERT(output_counter <= elementAABBs_outputs.size());

                            rtkd::KDTask task_L;
                            rtkd::KDTask task_R;

                            task_L.beg = alloc_head;
                            task_R.beg = alloc_head + best_nL;

                            //printf("%d -> %d %d\n", nElement, best_nL, best_nR);
                            float lMax = -FLT_MAX;
                            float rMin = +FLT_MAX;
                            int i_L = 0;
                            int i_R = 0;
                            for (int i = task.beg; i != task.end; i++)
                            {
                                rtkd::KDElement elem = elementAABBs_inputs[i];
                                int index_min = rtkd::bucket(elem.aabb.lower[best_axis], task.aabb.lower[best_axis], task.aabb.upper[best_axis], NBins);
                                int index_max = rtkd::bucket(elem.aabb.upper[best_axis], task.aabb.lower[best_axis], task.aabb.upper[best_axis], NBins);

                                rtkd::Triangle triangle = triangles[elem.triangleIndex];
                                rtkd::AABB clippedL, clippedR;
                                divide_clip(&clippedL, &clippedR, elem.aabb, triangle.vs[0], triangle.vs[1], triangle.vs[2], boundary, best_axis, 64);

                                clippedL = task.aabb.intersect(clippedL);
                                clippedR = task.aabb.intersect(clippedR);

                                if (index_min < best_i_split)
                                {
                                    if (clippedL.isEmpty() == false)
                                    {
                                        PR_ASSERT(clippedL.hasVolume());
                                        rtkd::KDElement clipped = elem;
                                        clipped.aabb = clippedL;
                                        elementAABBs_outputs[task_L.beg + i_L++] = clipped;

                                        lMax = rtkd::ss_max(lMax, elem.aabb.upper[best_axis]);
                                    }
                                }
                                if (best_i_split <= index_max)
                                {
                                    if (clippedR.isEmpty() == false)
                                    {
                                        PR_ASSERT(clippedR.hasVolume());
                                        rtkd::KDElement clipped = elem;
                                        clipped.aabb = clippedR;
                                        elementAABBs_outputs[task_R.beg + i_R++] = clipped;

                                        rMin = rtkd::ss_min(rMin, elem.aabb.lower[best_axis]);
                                    }
                                }
                            }

                            if (best_nL + best_nR == nElement) // meaning no overlap, update boundary to fit
                            {
                                PR_ASSERT(lMax <= rMin);

                                if (best_nL == 0)
                                {
                                    boundary = rMin;
                                }
                                else if (best_nR == 0)
                                {
                                    boundary = lMax;
                                }
                                else
                                {
                                    boundary = rMin;
                                }
                            }

                            //printf("L %d->%d\n", best_nL, i_L);
                            //printf("R %d->%d\n", best_nR, i_R);

                            best_aabbL.upper[best_axis] = boundary;
                            best_aabbR.lower[best_axis] = boundary;
                            task_L.aabb = best_aabbL;
                            task_R.aabb = best_aabbR;

                            task_L.end = task_L.beg + i_L;
                            task_R.end = task_R.beg + i_R;

                            // emit node
                            uint64_t nodeIndex = kdtreeBuffer.size() / rtkd::KD_ALIGNMENT;
                            kdtreeBuffer.resize(kdtreeBuffer.size() + sizeof(rtkd::KDNode));
                            rtkd::KDNode *node = (rtkd::KDNode *)&kdtreeBuffer[nodeIndex * rtkd::KD_ALIGNMENT];
                            node->axis = best_axis;
                            node->boundary = boundary;
                            node->childL = 0xFFFFFFFF;
                            node->childR = 0xFFFFFFFF;
                            if (task.slot_ptr != 0xFFFFFFFFFFFFFFFF)
                            {
                                uint32_t* slot = (uint32_t *)&kdtreeBuffer[task.slot_ptr];
                                *slot = (uint32_t)nodeIndex;
                            }

                            task_L.slot_ptr = (uint8_t*)&node->childL - kdtreeBuffer.data();
                            task_R.slot_ptr = (uint8_t*)&node->childR - kdtreeBuffer.data();

                            if (i_L)
                            {
                                PR_ASSERT(task_L.aabb.isEmpty() == false);

                                tasks_outputs.push_back(task_L);
                            }
                            if (i_R)
                            {
                                PR_ASSERT(task_R.aabb.isEmpty() == false);

                                tasks_outputs.push_back(task_R);
                            }

                            //PR_ASSERT(best_nL == i_L);
                            //PR_ASSERT(best_nR == i_R);

                            if (0 <= best_axis && debug_index == i_task)
                            {
                                // printf("{{%.20f, %.20f, %.20f}, {%.20f, %.20f, %.20f}};\n", task.aabb.lower[0], task.aabb.lower[1], task.aabb.lower[2], task.aabb.upper[0], task.aabb.upper[1], task.aabb.upper[2]);

                                //glm::vec3 axis_vector = {};
                                //axis_vector[best_axis] = 1.0f;
                                //glm::vec3 t0, t1;
                                //GetOrthonormalBasis(axis_vector, &t0, &t1);

                                //float b = rtkd::border(best_i_split, task.aabb.lower[best_axis], task.aabb.upper[best_axis], NBins);
                                //DrawFreeGrid(axis_vector * b, t0, t1, 2, { 200, 200, 200 });
                                //if (best_nL == 0 || best_nR == 0)
                                {
                                    //  && best_aabbL.volume() < 0.0000001f
                                    if (i_L)
                                    {
                                        //if (best_aabbL.volume() < 0.0)
                                        {
                                            //pr::PrimBegin(pr::PrimitiveMode::Lines, 3);

                                            //auto tri = triangles[elementAABBs_outputs[task_L.beg + 0].triangleIndex];
                                            //{
                                            //    for (int j = 0; j < 3; ++j)
                                            //    {
                                            //        rtkd::Vec3 v0 = tri.vs[j];
                                            //        rtkd::Vec3 v1 = tri.vs[(j + 1) % 3];
                                            //        pr::PrimVertex({ v0.xs[0], v0.xs[1], v0.xs[2] }, { 255, 0, 255 });
                                            //        pr::PrimVertex({ v1.xs[0], v1.xs[1], v1.xs[2] }, { 255, 0, 255 });
                                            //    }
                                            //}

                                            //pr::PrimEnd();

                                            DrawAABB(toGLM(best_aabbL.lower), toGLM(best_aabbL.upper), { 255, 0, 0 });
                                        }
                                    }

                                    // && best_aabbR.volume() < 0.0000001f
                                    if (i_R)
                                    {
                                        //pr::PrimBegin(pr::PrimitiveMode::Lines, 3);

                                        //for (int k = 0; k < i_R; k++)
                                        //{
                                        //    auto tri = triangles[elementAABBs_outputs[task_L.beg + k].triangleIndex];
                                        //    for (int j = 0; j < 3; ++j)
                                        //    {
                                        //        rtkd::Vec3 v0 = tri.vs[j];
                                        //        rtkd::Vec3 v1 = tri.vs[(j + 1) % 3];
                                        //        pr::PrimVertex({ v0.xs[0], v0.xs[1], v0.xs[2] }, { 255, 0, 255 });
                                        //        pr::PrimVertex({ v1.xs[0], v1.xs[1], v1.xs[2] }, { 255, 0, 255 });
                                        //    }
                                        //}

                                        //pr::PrimEnd();

                                        //if (best_aabbR.volume() < 0.0)
                                            DrawAABB(toGLM(best_aabbR.lower), toGLM(best_aabbR.upper), { 0, 255, 0 });
                                    }
                                    //DrawAABB(toGLM(task.aabb.lower) * 1.001f, toGLM(task.aabb.upper) * 1.001f, { 255, 255, 255 });
                                }

                                //for (int j = task_L.beg; j < task_L.end; j++)
                                //{
                                //    printf("vl %f\n", elementAABBs_outputs[j].aabb.volume());
                                //    DrawAABB(toGLM(elementAABBs_outputs[j].aabb.lower), toGLM(elementAABBs_outputs[j].aabb.upper), { 128, 0, 0 });
                                //}
                                //for (int j = task_R.beg; j < task_R.end; j++)
                                //{
                                //    printf("vr %f\n", elementAABBs_outputs[j].aabb.volume());

                                //    DrawAABB(toGLM(elementAABBs_outputs[j].aabb.lower), toGLM(elementAABBs_outputs[j].aabb.upper), { 0, 128, 0 });
                                //}
                            }
                        }
                        else
                        {
                            uint64_t nodeIndex = kdtreeBuffer.size() / rtkd::KD_ALIGNMENT;
                            kdtreeBuffer.resize(kdtreeBuffer.size() + sizeof(rtkd::KDLeaf));
                            rtkd::KDLeaf* node = (rtkd::KDLeaf*)&kdtreeBuffer[nodeIndex * rtkd::KD_ALIGNMENT];

                            // triangleBuffer
                            uint32_t nTriangles = task.end - task.beg;
                            node->triangleBeg = triangleBuffer.size();
                            triangleBuffer.resize(triangleBuffer.size() + nTriangles);
                            node->triangleEnd = node->triangleBeg + nTriangles;

                            for (int i = 0; i != nTriangles; i++)
                            {
                                triangleBuffer[node->triangleBeg + i] = elementAABBs_inputs[task.beg + i].triangleIndex;
                            }

                            if (task.slot_ptr != 0xFFFFFFFFFFFFFFFF)
                            {
                                uint32_t* slot = (uint32_t*)&kdtreeBuffer[task.slot_ptr];
                                *slot = (uint32_t)nodeIndex | rtkd::KD_LEAF_FLAG;
                            }
                        }
                    }

                    if (tasks_outputs.empty())
                    {
                        break;
                    }

                    printf("stage %d, o=%d / %d tri\n", i_task, output_counter, (int)triangles.size());

                    tasks_inputs.clear();
                    std::swap(tasks_inputs, tasks_outputs);
                    std::swap(elementAABBs_inputs, elementAABBs_outputs);
                }

            }


            rtkd::KDHit hit;
            rtkd::intersect(&hit, { ro.x, ro.y, ro.z }, { rd.x, rd.y, rd.z }, sceneBounds, kdtreeBuffer.data(), triangleBuffer.data(), triangles.data());

            if (hit.t != FLT_MAX)
            {
                auto hitp = toGLM(ro + rd * hit.t);
                auto ng = glm::normalize(toGLM(hit.ng));
                pr::DrawSphere(hitp, 0.02f, { 255,0,255 });
                pr::DrawArrow(hitp, hitp + ng * 0.2f, 0.01f, { 128 , 128 ,128 });
            }

#if 1
            int stride = 1;
            Image2DRGBA8 image;
		    image.allocate( GetScreenWidth() / stride, GetScreenHeight() / stride);

		    CameraRayGenerator rayGenerator( GetCurrentViewMatrix(), GetCurrentProjMatrix(), image.width(), image.height() );

		    for( int j = 0; j < image.height(); ++j )
		    {
			    for( int i = 0; i < image.width(); ++i )
			    {
				    glm::vec3 ro, rd;
				    rayGenerator.shoot( &ro, &rd, i, j, 0.5f, 0.5f );

                    rtkd::KDHit hit;
                    rtkd::intersect(&hit, { ro.x, ro.y, ro.z }, { rd.x, rd.y, rd.z }, sceneBounds, kdtreeBuffer.data(), triangleBuffer.data(), triangles.data());

				    if( hit.t != FLT_MAX )
				    {
                        glm::vec3 n = toGLM(hit.ng);
					    n = glm::normalize( n );

					    glm::vec3 color = ( n + glm::vec3( 1.0f ) ) * 0.5f;
					    image( i, j ) = { 255 * color.r, 255 * color.g, 255 * color.b, 255 };
				    }
				    else
				    {
					    image( i, j ) = { 0, 0, 0, 255 };
				    }
			    }
		    }

            texture->upload( image );
#endif
        });


        PopGraphicState();
        EndCamera();

        BeginImGui();

        ImGui::SetNextWindowSize({ 500, 800 }, ImGuiCond_Once);
        ImGui::Begin("Panel");
        ImGui::Text("fps = %f", GetFrameRate());
        ImGui::InputInt("debug index", &debug_index);

        ImGui::Checkbox("showWire", &showWire);
        if (ImGui::Button("dump"))
        {
            printf("ro = {%f, %f, %f}\n", ro.x, ro.y, ro.z);
            printf("to = {%f, %f, %f}\n", to.x, to.y, to.z);
        }
        ImGui::End();

        EndImGui();
    }

    pr::CleanUp();
}
