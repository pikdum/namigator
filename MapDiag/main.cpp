// MapDiag — navmesh connectivity diagnostic.
//
// Loads a pre-built MapBuilder output for a given map and analyzes the raw
// Detour polygon connectivity, without going through the query API. The point
// is to answer: "is this navmesh one connected surface, or is it fragmented
// into disconnected islands?" — because pathfinding can only route between two
// polygons that share a connected component. Fragmentation (especially tiles
// whose shared borders never linked) is the signature of the caves / global-WMO
// path failures.
//
// Usage:
//   MapDiag <nav_data_dir> <MapName> [MapName...] [--top=N] [--tiles] [--no-adts]
//
// Example:
//   MapDiag /home/pikdum/code/thistle_tea/maps OrgrimmarInstance Stratholme

#include "pathfind/Map.hpp"
#include "recastnavigation/Detour/Include/DetourNavMesh.h"
#include "utility/BoundingBox.hpp"
#include "utility/MathHelper.hpp"
#include "utility/Vector.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
// Union-find over polygon nodes, keyed by dense index.
class DisjointSet
{
public:
    explicit DisjointSet(int n) : m_parent(n), m_size(n, 1)
    {
        for (int i = 0; i < n; ++i)
            m_parent[i] = i;
    }

    int Find(int x)
    {
        while (m_parent[x] != x)
        {
            m_parent[x] = m_parent[m_parent[x]];
            x = m_parent[x];
        }
        return x;
    }

    void Union(int a, int b)
    {
        a = Find(a);
        b = Find(b);
        if (a == b)
            return;
        if (m_size[a] < m_size[b])
            std::swap(a, b);
        m_parent[b] = a;
        m_size[a] += m_size[b];
    }

private:
    std::vector<int> m_parent;
    std::vector<int> m_size;
};

struct Options
{
    int top = 8;
    bool dumpTiles = false;
    bool loadAdts = true;
    bool hasNear = false;   // restrict ADT loading to a point
    bool hasNearZ = false;  // a Z was supplied -> probe the component there
    double nearX = 0.0;
    double nearY = 0.0;
    double nearZ = 0.0;
    double radius = 150.0;  // world units around the point to cover
};

const char* PolyFlagName(int bit)
{
    switch (bit)
    {
        case 0: return "Ground";
        case 1: return "Steep";
        case 2: return "Liquid";
        case 3: return "Wmo";
        case 4: return "Doodad";
        default: return "?";
    }
}

void Analyze(const std::string& dataDir, const std::string& mapName,
             const Options& opts)
{
    std::printf("\n=== %s ===\n", mapName.c_str());

    std::unique_ptr<pathfind::Map> map;
    try
    {
        map = std::make_unique<pathfind::Map>(dataDir, mapName);
    }
    catch (const std::exception& e)
    {
        std::printf("  FAILED to load: %s\n", e.what());
        return;
    }

    const bool hasAdts = map->HasADTs();
    if (hasAdts)
    {
        std::printf("  type:            ADT-based\n");
        if (!opts.loadAdts)
        {
            std::printf("  ADTs loaded:     skipped (--no-adts)\n");
        }
        else if (opts.hasNear)
        {
            // Convert the four corners of the [point +/- radius] box to ADT
            // indices and load every existing ADT in that range. Going via the
            // corners handles the world<->ADT axis swap/sign automatically.
            int minAx = MeshSettings::Adts, maxAx = -1;
            int minAy = MeshSettings::Adts, maxAy = -1;
            for (int cx = 0; cx < 2; ++cx)
                for (int cy = 0; cy < 2; ++cy)
                {
                    int ax, ay;
                    math::Convert::WorldToAdt(
                        {static_cast<float>(opts.nearX +
                                            (cx ? opts.radius : -opts.radius)),
                         static_cast<float>(opts.nearY +
                                            (cy ? opts.radius : -opts.radius)),
                         0.f},
                        ax, ay);
                    minAx = std::min(minAx, ax);
                    maxAx = std::max(maxAx, ax);
                    minAy = std::min(minAy, ay);
                    maxAy = std::max(maxAy, ay);
                }
            int loaded = 0;
            for (int ax = std::max(0, minAx);
                 ax <= std::min(MeshSettings::Adts - 1, maxAx); ++ax)
                for (int ay = std::max(0, minAy);
                     ay <= std::min(MeshSettings::Adts - 1, maxAy); ++ay)
                    if (map->HasADT(ax, ay) && map->LoadADT(ax, ay))
                        ++loaded;
            std::printf("  near:            (%.0f, %.0f) r=%.0f -> ADT x[%d..%d] "
                        "y[%d..%d]\n",
                        opts.nearX, opts.nearY, opts.radius, minAx, maxAx, minAy,
                        maxAy);
            std::printf("  ADTs loaded:     %d\n", loaded);
            if (loaded == 0)
            {
                std::printf("  (no ADTs cover that point — nothing to analyze)\n");
                return;
            }
        }
        else
        {
            const auto start = std::chrono::steady_clock::now();
            const int loaded = map->LoadAllADTs();
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - start)
                                .count();
            std::printf("  ADTs loaded:     %d (%lld ms)\n", loaded,
                        static_cast<long long>(ms));
        }
    }
    else
    {
        std::printf("  type:            global WMO (no ADTs)\n");
    }

    const dtNavMesh& nav = map->GetNavMesh();
    const int maxTiles = nav.getMaxTiles();

    // First pass: give every polygon a dense index, remember its owning tile.
    std::unordered_map<dtPolyRef, int> refToIdx;
    std::vector<int> nodeTile;    // tile loop-index (0..maxTiles) per node
    std::vector<int> nodeType;    // dtPolyType per node
    std::vector<int> nodeFlags;   // dtPoly::flags per node
    std::vector<math::Vertex> nodePos;  // poly centroid in WoW coordinates
    int loadedTiles = 0;
    for (int t = 0; t < maxTiles; ++t)
    {
        const dtMeshTile* tile = nav.getTile(t);
        if (!tile || !tile->header)
            continue;
        ++loadedTiles;
        const dtPolyRef base = nav.getPolyRefBase(tile);
        for (int i = 0; i < tile->header->polyCount; ++i)
        {
            const dtPoly& poly = tile->polys[i];
            const dtPolyRef ref = base | static_cast<dtPolyRef>(i);
            refToIdx[ref] = static_cast<int>(nodeTile.size());
            nodeTile.push_back(t);
            nodeType.push_back(poly.getType());
            nodeFlags.push_back(poly.flags);

            // centroid of the polygon's vertices, converted to WoW coordinates
            float recast[3] = {0.f, 0.f, 0.f};
            const int vc = poly.vertCount;
            for (int j = 0; j < vc; ++j)
            {
                const float* v = &tile->verts[poly.verts[j] * 3];
                recast[0] += v[0];
                recast[1] += v[1];
                recast[2] += v[2];
            }
            if (vc > 0)
            {
                recast[0] /= vc;
                recast[1] /= vc;
                recast[2] /= vc;
            }
            math::Vertex wow;
            math::Convert::VertexToWow(recast, wow);
            nodePos.push_back(wow);
        }
    }

    const int n = static_cast<int>(nodeTile.size());
    if (n == 0)
    {
        std::printf("  polygons:        0 (empty navmesh)\n");
        return;
    }

    // Second pass: union polygons across every Detour link. Links are stored on
    // both endpoints, so each undirected adjacency is visited twice.
    DisjointSet dsu(n);
    long long crossTileLinks = 0;
    long long intraTileLinks = 0;
    std::vector<char> tileHasCrossLink(maxTiles, 0);
    for (int t = 0; t < maxTiles; ++t)
    {
        const dtMeshTile* tile = nav.getTile(t);
        if (!tile || !tile->header)
            continue;
        const dtPolyRef base = nav.getPolyRefBase(tile);
        for (int i = 0; i < tile->header->polyCount; ++i)
        {
            const int a = refToIdx[base | static_cast<dtPolyRef>(i)];
            for (unsigned int k = tile->polys[i].firstLink; k != DT_NULL_LINK;
                 k = tile->links[k].next)
            {
                const dtPolyRef neighbor = tile->links[k].ref;
                if (!neighbor)
                    continue;
                const auto it = refToIdx.find(neighbor);
                if (it == refToIdx.end())
                    continue;  // neighbor lives in an unloaded tile
                const int b = it->second;
                if (nodeTile[a] != nodeTile[b])
                {
                    ++crossTileLinks;
                    tileHasCrossLink[nodeTile[a]] = 1;
                }
                else
                {
                    ++intraTileLinks;
                }
                dsu.Union(a, b);
            }
        }
    }

    // Tally polygon types and flags.
    int offMeshPolys = 0;
    long long flagCounts[8] = {0};
    for (int i = 0; i < n; ++i)
    {
        if (nodeType[i] == DT_POLYTYPE_OFFMESH_CONNECTION)
            ++offMeshPolys;
        for (int b = 0; b < 8; ++b)
            if (nodeFlags[i] & (1 << b))
                ++flagCounts[b];
    }

    // Component sizes, spanned tiles, and WoW-space bounding box.
    std::unordered_map<int, int> compSize;
    std::unordered_map<int, std::unordered_set<int>> compTiles;
    std::unordered_map<int, math::BoundingBox> compBox;
    for (int i = 0; i < n; ++i)
    {
        const int root = dsu.Find(i);
        ++compSize[root];
        compTiles[root].insert(nodeTile[i]);
        const math::Vertex& p = nodePos[i];
        const auto it = compBox.find(root);
        if (it == compBox.end())
            compBox.emplace(root, math::BoundingBox(p, p));
        else
        {
            math::BoundingBox& b = it->second;
            b.MinCorner.X = std::min(b.MinCorner.X, p.X);
            b.MinCorner.Y = std::min(b.MinCorner.Y, p.Y);
            b.MinCorner.Z = std::min(b.MinCorner.Z, p.Z);
            b.MaxCorner.X = std::max(b.MaxCorner.X, p.X);
            b.MaxCorner.Y = std::max(b.MaxCorner.Y, p.Y);
            b.MaxCorner.Z = std::max(b.MaxCorner.Z, p.Z);
        }
    }

    std::vector<std::pair<int, int>> comps;  // (size, root)
    comps.reserve(compSize.size());
    int singletons = 0;
    for (const auto& kv : compSize)
    {
        comps.emplace_back(kv.second, kv.first);
        if (kv.second == 1)
            ++singletons;
    }
    std::sort(comps.begin(), comps.end(),
              [](const std::pair<int, int>& l, const std::pair<int, int>& r)
              { return l.first > r.first; });

    int isolatedTiles = 0;
    for (int t = 0; t < maxTiles; ++t)
    {
        const dtMeshTile* tile = nav.getTile(t);
        if (!tile || !tile->header)
            continue;
        if (!tileHasCrossLink[t])
            ++isolatedTiles;
    }

    const double largestPct =
        100.0 * static_cast<double>(comps.front().first) / static_cast<double>(n);

    math::Vertex meshLo = nodePos[0];
    math::Vertex meshHi = nodePos[0];
    for (const auto& p : nodePos)
    {
        meshLo.X = std::min(meshLo.X, p.X);
        meshLo.Y = std::min(meshLo.Y, p.Y);
        meshLo.Z = std::min(meshLo.Z, p.Z);
        meshHi.X = std::max(meshHi.X, p.X);
        meshHi.Y = std::max(meshHi.Y, p.Y);
        meshHi.Z = std::max(meshHi.Z, p.Z);
    }

    std::printf("  tiles loaded:    %d\n", loadedTiles);
    std::printf("  polygons:        %d  (offmesh %d)\n", n, offMeshPolys);
    std::printf("  mesh bbox:       x[%.0f..%.0f] y[%.0f..%.0f] z[%.0f..%.0f]\n",
                meshLo.X, meshHi.X, meshLo.Y, meshHi.Y, meshLo.Z, meshHi.Z);
    std::printf("  flags:           ");
    for (int b = 0; b < 5; ++b)
        std::printf("%s=%lld ", PolyFlagName(b), flagCounts[b]);
    std::printf("\n");
    std::printf("  edges:           %lld intra-tile, %lld cross-tile\n",
                intraTileLinks / 2, crossTileLinks / 2);
    if (loadedTiles > 1)
        std::printf("  isolated tiles:  %d / %d have no cross-tile link\n",
                    isolatedTiles, loadedTiles);
    std::printf("  components:      %d  (singletons %d)\n",
                static_cast<int>(comps.size()), singletons);

    auto printComp = [&](int rank)
    {
        const int size = comps[rank].first;
        const int root = comps[rank].second;
        const double pct = 100.0 * size / n;
        const math::BoundingBox& b = compBox[root];
        std::printf("    #%-2d %8d polys = %5.1f%%  %3d tiles  z[%.0f..%.0f]\n",
                    rank + 1, size, pct,
                    static_cast<int>(compTiles[root].size()), b.MinCorner.Z,
                    b.MaxCorner.Z);
    };

    const int shown = std::min(opts.top, static_cast<int>(comps.size()));
    for (int c = 0; c < shown; ++c)
        printComp(c);

    // Do the other large regions sit above/below the largest (stacked levels),
    // or beside it (spatially separate)? Overlap in the horizontal (X,Y) plane
    // while separated in Z is the signature of levels that failed to link.
    auto xyOverlap = [](const math::BoundingBox& a, const math::BoundingBox& b)
    {
        return a.MinCorner.X <= b.MaxCorner.X && b.MinCorner.X <= a.MaxCorner.X &&
               a.MinCorner.Y <= b.MaxCorner.Y && b.MinCorner.Y <= a.MaxCorner.Y;
    };
    const math::BoundingBox& box0 = compBox[comps.front().second];
    int bigRegions = 0;
    int overlapping = 0;
    for (int c = 1; c < static_cast<int>(comps.size()); ++c)
    {
        if (100.0 * comps[c].first / n < 2.0)
            break;  // only consider substantial regions
        ++bigRegions;
        if (xyOverlap(box0, compBox[comps[c].second]))
            ++overlapping;
    }
    if (bigRegions > 0)
        std::printf("  big regions:     %d over 2%% of mesh; %d overlap the "
                    "largest horizontally\n",
                    bigRegions, overlapping);

    if (opts.dumpTiles)
    {
        std::printf("  per-tile (grid x,y : polys, cross-link?):\n");
        for (int t = 0; t < maxTiles; ++t)
        {
            const dtMeshTile* tile = nav.getTile(t);
            if (!tile || !tile->header)
                continue;
            std::printf("    (%3d,%3d): %5d polys  %s\n", tile->header->x,
                        tile->header->y, tile->header->polyCount,
                        tileHasCrossLink[t] ? "" : "<ISOLATED>");
        }
    }

    // Probe: which connected component does the --near point actually sit in?
    // This is the "can a mob standing here reach the rest of the world" test.
    if (opts.hasNear && opts.hasNearZ)
    {
        const dtNavMeshQuery& query = map->GetNavMeshQuery();
        dtQueryFilter filter;  // default include/exclude == namigator's FindPath
        const math::Vertex probe{static_cast<float>(opts.nearX),
                                 static_cast<float>(opts.nearY),
                                 static_cast<float>(opts.nearZ)};
        float center[3];
        math::Convert::VertexToRecast(probe, center);
        const float extents[3] = {12.f, 40.f, 12.f};
        dtPolyRef ref = 0;
        query.findNearestPoly(center, extents, &filter, &ref, nullptr);
        if (!ref)
            std::printf("  probe point:     (%.0f,%.0f,%.0f) has NO navmesh "
                        "within reach — unwalkable / ungenerated here\n",
                        opts.nearX, opts.nearY, opts.nearZ);
        else
        {
            const auto it = refToIdx.find(ref);
            if (it == refToIdx.end())
                std::printf("  probe point:     poly found but outside loaded "
                            "tiles (widen --radius)\n");
            else
            {
                const int root = dsu.Find(it->second);
                int rank = 0;
                for (int c = 0; c < static_cast<int>(comps.size()); ++c)
                    if (comps[c].second == root)
                    {
                        rank = c;
                        break;
                    }
                std::printf("  probe point:     in component #%d (%d polys = "
                            "%.1f%% of loaded mesh)%s\n",
                            rank + 1, compSize[root], 100.0 * compSize[root] / n,
                            rank == 0 ? " — the main surface" : " — ISOLATED "
                                                                "from main");
            }
        }
    }

    // Heuristic verdict. Many tiny stray islands are normal on large maps, so
    // key off the largest component's coverage, not the raw component count.
    if (loadedTiles > 1 && crossTileLinks == 0)
        std::printf("  VERDICT:         tile seams NOT linked — every tile is "
                    "its own island (addressing/border bug)\n");
    else if (largestPct >= 85.0)
        std::printf("  VERDICT:         healthy — one dominant surface (%.0f%%)"
                    ", rest is normal stray-island noise\n",
                    largestPct);
    else if (bigRegions > 0 && overlapping * 2 >= bigRegions)
        std::printf("  VERDICT:         FRAGMENTED — %d/%d big regions occupy "
                    "the SAME footprint as the largest yet don't connect "
                    "(connectors never built — suspect WalkableClimb / mesh "
                    "gaps)\n",
                    overlapping, bigRegions);
    else if (bigRegions > 0 && overlapping == 0)
        std::printf("  VERDICT:         split into spatially-separate regions — "
                    "expected if the map has disjoint areas, else fragmented\n");
    else
        std::printf("  VERDICT:         FRAGMENTED — largest surface only "
                    "%.0f%% of mesh\n",
                    largestPct);
}
}  // namespace

int main(int argc, char** argv)
{
    Options opts;
    std::string dataDir;
    std::vector<std::string> maps;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg.rfind("--top=", 0) == 0)
            opts.top = std::max(1, std::atoi(arg.c_str() + 6));
        else if (arg == "--tiles")
            opts.dumpTiles = true;
        else if (arg == "--no-adts")
            opts.loadAdts = false;
        else if (arg.rfind("--near=", 0) == 0)
        {
            double v[3] = {0, 0, 0};
            const int nv =
                std::sscanf(arg.c_str() + 7, "%lf,%lf,%lf", &v[0], &v[1], &v[2]);
            if (nv < 2)
            {
                std::fprintf(stderr, "bad --near (need X,Y[,Z])\n");
                return 2;
            }
            opts.hasNear = true;
            opts.nearX = v[0];
            opts.nearY = v[1];
            if (nv >= 3)
            {
                opts.hasNearZ = true;
                opts.nearZ = v[2];
            }
        }
        else if (arg.rfind("--radius=", 0) == 0)
            opts.radius = std::atof(arg.c_str() + 9);
        else if (arg.rfind("--", 0) == 0)
        {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            return 2;
        }
        else if (dataDir.empty())
            dataDir = arg;
        else
            maps.push_back(arg);
    }

    if (dataDir.empty() || maps.empty())
    {
        std::fprintf(stderr,
                     "usage: MapDiag <nav_data_dir> <MapName> [MapName...] "
                     "[--top=N] [--tiles] [--no-adts]\n"
                     "                [--near=X,Y[,Z]] [--radius=R]\n"
                     "  --near restricts ADT loading to a point (for spot-"
                     "checking one cave on a continent);\n"
                     "  a Z also reports which connected component that point "
                     "sits in.\n");
        return 2;
    }

    for (const auto& m : maps)
        Analyze(dataDir, m, opts);

    return 0;
}
