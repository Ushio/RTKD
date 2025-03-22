#include "pr.hpp"
#include <iostream>
#include <memory>

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

    struct Triangle
    {
        Vec3 vs[3];
    };

    struct AABB
    {
        Vec3 lower;
        Vec3 upper;

        void set_empty()
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
    };

    struct KDTask
    {
        int beg;
        int end;
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

    static const float COST_TRAVERSE = 1.0f;
    static const float COST_INTERSECT = 4.0f;
}

inline glm::vec3 toGLM(rtkd::Vec3 x)
{
    return { x[0], x[1], x[2] };
}

int main() {
    using namespace pr;

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

        static int debug_index = 0;

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
            std::vector<rtkd::AABB> elementAABBs(triangles.size());
            rtkd::AABB box;
            box.set_empty();
            for (int i = 0; i < triangles.size(); i++)
            {
                elementAABBs[i].set_empty();
                for (int j = 0; j < 3; j++)
                {
                    elementAABBs[i].extend(triangles[i].vs[j]);
                    box.extend(triangles[i].vs[j]);
                }
            }

            std::vector<int> elements_inputs(elementAABBs.size());
            std::vector<int> elements_outputs;

            for (int i = 0; i < elementAABBs.size(); i++)
            {
                elements_inputs[i] = i;
            }
            std::vector<rtkd::KDTask> tasks_inputs;
            std::vector<rtkd::KDTask> tasks_outputs;
            tasks_inputs.push_back(
                {
                    0, (int)elementAABBs.size(),
                    box
                }
            );

            for (rtkd::KDTask task : tasks_inputs)
            {
                int best_i_split = -1;
                float best_cost = (task.end - task.beg) * rtkd::COST_INTERSECT;

                enum { NBins = 32 };
                int min_bins[NBins] = {};
                int max_bins[NBins] = {};

                rtkd::AABB min_AABB[NBins];
                rtkd::AABB max_AABB[NBins];
                for (int i = 0; i < NBins; i++)
                {
                    min_AABB[i].set_empty();
                    max_AABB[i].set_empty();
                }

                int axis = 0;
                for (int i = task.beg; i != task.end; i++)
                {
                    rtkd::AABB box = elementAABBs[i];
                    int index_min = rtkd::bucket(box.lower[axis], task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                    int index_max = rtkd::bucket(box.upper[axis], task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                    min_bins[index_min]++;
                    max_bins[index_max]++;
                    min_AABB[index_min].extend(box);
                    max_AABB[index_max].extend(box);
                }

                // inclusive prefix sum
                rtkd::AABB mi; mi.set_empty();
                rtkd::AABB ma; ma.set_empty();
                int mi_count = 0;
                int ma_count = 0;
                for (int i = 0; i < NBins; i++)
                {
                    mi.extend(min_AABB[i]);
                    min_AABB[i] = mi;

                    ma.extend(max_AABB[NBins - i - 1]);
                    max_AABB[NBins - i - 1] = ma;

                    mi_count += min_bins[i];
                    min_bins[i] = mi_count;

                    ma_count += max_bins[NBins - i - 1];
                    max_bins[NBins - i - 1] = ma_count;
                }

                float p_area = task.aabb.surface_area();
                for (int i_split = 1; i_split < NBins; i_split++)
                {
                    rtkd::AABB L = min_AABB[i_split - 1];
                    rtkd::AABB R = max_AABB[i_split];
                    int nL = min_bins[i_split - 1];
                    int nR = max_bins[i_split];

                    float cost = rtkd::COST_TRAVERSE +
                        (L.surface_area() / p_area) * nL * rtkd::COST_INTERSECT +
                        (R.surface_area() / p_area) * nR * rtkd::COST_INTERSECT;

                    if (cost < best_cost)
                    {
                        best_cost = cost;
                        best_i_split = i_split;
                    }
                }

                int i_split = best_i_split;
                 //int i_split = debug_index + 1;
                float b = rtkd::border(i_split, task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                DrawFreeGrid({ b, 0, 0 }, { 0, 0.5f, 0 }, { 0, 0, 0.5f }, 2, { 128 ,128 ,128 });

                DrawAABB(toGLM(min_AABB[i_split - 1].lower), toGLM(min_AABB[i_split - 1].upper), { 255, 0, 0 });
                DrawAABB(toGLM(max_AABB[i_split].lower), toGLM(max_AABB[i_split].upper), { 0, 255, 0 });
                printf("%d-%d\n", min_bins[i_split - 1], max_bins[i_split]);

                //DrawAABB(toGLM(min_AABB[debug_index].lower), toGLM(min_AABB[debug_index].upper), { 255, 0, 0 });

                //float b0 = rtkd::border(debug_index, task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                //float b1 = rtkd::border(debug_index + 1, task.aabb.lower[axis], task.aabb.upper[axis], NBins);
                //DrawFreeGrid({ b0, 0, 0 }, { 0, 0.5f, 0 }, { 0, 0, 0.5f }, 2, { 128 ,128 ,128 });
                //DrawFreeGrid({ b1, 0, 0 }, { 0, 0.5f, 0 }, { 0, 0, 0.5f }, 2, { 128 ,128 ,128 });
            }
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
