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

    struct Triangle
    {
        Vec3 vs[3];
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
        //void extend(const AABB& b)
        //{
        //    for (int i = 0; i < 3; i++)
        //    {
        //        lower[i] = ss_min(lower[i], b.lower[i]);
        //        upper[i] = ss_max(upper[i], b.upper[i]);
        //    }
        //}
        //void extend(const AABB& b, int axis)
        //{
        //    lower[axis] = ss_min(lower[axis], b.lower[axis]);
        //    upper[axis] = ss_max(upper[axis], b.upper[axis]);
        //}
        float surface_area() const
        {
            Vec3 size = upper - lower;
            return (size[0] * size[1] + size[1] * size[2] + size[2] * size[0]) * 2.0f;
        }
        float volume() const
        {
            Vec3 size = upper - lower;
            return size[0] * size[1] * size[2];
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
                if (upper[i] < lower[i])
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
    };
    struct KDElement
    {
        int triangleIndex;
        AABB aabb;
    };

    inline int bucket( float x, float lower, float upper, int nBins )
    {
        int index = (int)( (x - lower) * nBins / (upper - lower) );
        return ss_min(ss_max(0, index), nBins - 1);
    }

    // i=0: lower, i=nBins: upper
    inline float border(int i, float lower, float upper, int nBins)
    {
        return lower + (upper - lower) / nBins * i;
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

            // move p for conservative clipping
            //if (-FLT_EPSILON * nEps < t && t < 1.0f + FLT_EPSILON * nEps)
            if (-FLT_EPSILON * nEps < t && t < 1.0f + FLT_EPSILON * nEps)
            {
                Vec3 p = ro + rd * t;
                float bias = ss_max(fabsf(p[axis] * FLT_EPSILON), FLT_MIN) * nEps;
                Vec3 pL = p;
                Vec3 pR = p;
                pL[axis] += bias;
                pR[axis] -= bias;

                L->extend(pL);
                R->extend(pR);
            }
        }

        *L = baseAABB.intersect(*L);
        *R = baseAABB.intersect(*R);
    }

    static const float COST_TRAVERSE = 1.0f;
    static const float COST_INTERSECT = 8.0f;

    struct KDNode
    {

    };
}


void test()
{

}

int main() {
    using namespace pr;

    test();

    Config config;
    config.ScreenWidth = 1920;
    config.ScreenHeight = 1080;
    config.SwapInterval = 1;
    Initialize(config);

    Camera3D camera;
    camera.origin = { 4, 4, 4 };
    camera.lookat = { 0, 0, 0 };

    double e = GetElapsedTime();

    SetDataDir(ExecutableDir());
    std::string err;
    std::shared_ptr<FScene> scene = ReadWavefrontObj(GetDataPath("test.obj"), err);

    while (pr::NextFrame() == false) {
        if (IsImGuiUsingMouse() == false) {
            UpdateCameraBlenderLike(&camera);
        }

        ClearBackground(0.1f, 0.1f, 0.1f, 1);

        BeginCamera(camera);

        PushGraphicState();

        DrawGrid(GridAxis::XZ, 1.0f, 10, { 128, 128, 128 });
        DrawXYZAxis(1.0f);

        // hmm still some bad aabb..
        static int debug_index = 8;

        scene->visitPolyMesh([](std::shared_ptr<const FPolyMeshEntity> polymesh) {
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
                triangles.push_back(tri);
                indexBase += nVerts;
            }

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

            // Build KD Tree
            std::vector<rtkd::KDElement> elementAABBs_inputs(triangles.size() * 8);
            std::vector<rtkd::KDElement> elementAABBs_outputs(triangles.size() * 8);

            rtkd::AABB box;
            box.setEmpty();
            for (int i = 0; i < triangles.size(); i++)
            {
                rtkd::AABB triangleAABB;
                triangleAABB.setEmpty();
                for (int j = 0; j < 3; j++)
                {
                    triangleAABB.extend(triangles[i].vs[j]);
                    box.extend(triangles[i].vs[j]);
                }
                elementAABBs_inputs[i].triangleIndex = i;
                elementAABBs_inputs[i].aabb = triangleAABB;
            }

            std::vector<rtkd::KDTask> tasks_inputs;
            std::vector<rtkd::KDTask> tasks_outputs;
            tasks_inputs.push_back(
                {
                    0, (int)triangles.size(),
                    box
                }
            );

            int maxDepth = 8 + log2(triangles.size());

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
                    float best_maxL = 0.0f;
                    float best_minR = 0.0f;
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
                        int min_bins[NBins] = {};
                        int max_bins[NBins] = {};

                        float max_L = -FLT_MAX;
                        float min_R = +FLT_MAX;

                        for (int i = task.beg; i != task.end; i++)
                        {
                            rtkd::AABB box = elementAABBs_inputs[i].aabb;
                            int index_min = rtkd::bucket(box.lower[axis], task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                            int index_max = rtkd::bucket(box.upper[axis], task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                            min_bins[index_min]++;
                            max_bins[index_max]++;
                            max_L = rtkd::ss_max(max_L, box.upper[axis]);
                            min_R = rtkd::ss_min(min_R, box.lower[axis]);
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
                            if (cost < best_cost )
                            {
                                best_cost = cost;
                                best_i_split = i_split;
                                best_axis = axis;
                                best_nL = nL;
                                best_nR = nR;
                                best_aabbL = L;
                                best_aabbR = R;
                                best_maxL = max_L;
                                best_minR = min_R;
                            }
                        }
                    }

                    if (0 < best_i_split)
                    {
                        // update boundary 
                        float boundary = rtkd::border(best_i_split, task.aabb.lower[best_axis], task.aabb.upper[best_axis], NBins);
                        if(best_nL == 0)
                        {
                            boundary = best_minR;
                        }
                        if (best_nR == 0)
                        {
                            boundary = best_maxL;
                        }
                        best_aabbL.upper[best_axis] = boundary;
                        best_aabbR.lower[best_axis] = boundary;

                        int alloc_head = output_counter;
                        output_counter += best_nL + best_nR;

                        PR_ASSERT(output_counter <= elementAABBs_outputs.size());

                        rtkd::KDTask task_L;
                        rtkd::KDTask task_R;

                        task_L.beg = output_counter;
                        //task_L.end = output_counter + best_nL;
                        task_L.aabb = best_aabbL;
                        task_R.beg = output_counter + best_nL;
                        //task_R.end = output_counter + best_nL + best_nR;
                        task_R.aabb = best_aabbR;

                        printf("%d -> %d %d\n", nElement, best_nL, best_nR);

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

                            if (index_min < best_i_split)
                            {
                                if (clippedL.isEmpty() == false)
                                {
                                    PR_ASSERT(0.0f < clippedL.volume());
                                    rtkd::KDElement clipped = elem;
                                    clipped.aabb = clippedL;
                                    elementAABBs_outputs[task_L.beg + i_L++] = clipped;
                                }
                            }
                            if (best_i_split <= index_max)
                            {
                                if (clippedR.isEmpty() == false)
                                {
                                    PR_ASSERT(0.0f < clippedR.volume());
                                    rtkd::KDElement clipped = elem;
                                    clipped.aabb = clippedR;
                                    elementAABBs_outputs[task_R.beg + i_R++] = clipped;
                                }
                            }
                        }

                        task_L.end = task_L.beg + i_L;
                        task_R.end = task_R.beg + i_R;

                        if (i_L)
                        {
                            tasks_outputs.push_back(task_L);
                        }
                        if (i_R)
                        {
                            tasks_outputs.push_back(task_R);
                        }
                        //PR_ASSERT(best_nL == i_L);
                        //PR_ASSERT(best_nR == i_R);

                        if (0 <= best_axis && debug_index == i_task)
                        {
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
                                    DrawAABB(toGLM(best_aabbL.lower), toGLM(best_aabbL.upper), { 255, 0, 0 });
                                }

                                // && best_aabbR.volume() < 0.0000001f
                                if (i_R)
                                {
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
                }

                if (tasks_outputs.empty())
                {
                    break;
                }

                printf("stage %d, o=%d\n", i_task, output_counter);

                tasks_inputs.clear();
                std::swap(tasks_inputs, tasks_outputs);
                std::swap(elementAABBs_inputs, elementAABBs_outputs);
            }

            printf("");
        });

        PopGraphicState();
        EndCamera();

        BeginImGui();

        ImGui::SetNextWindowSize({ 500, 800 }, ImGuiCond_Once);
        ImGui::Begin("Panel");
        ImGui::Text("fps = %f", GetFrameRate());
        ImGui::InputInt("debug index", &debug_index);

        ImGui::End();

        EndImGui();
    }

    pr::CleanUp();
}
