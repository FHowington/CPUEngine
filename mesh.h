//#include <cstdio>
#include <vector>
#include <random>

auto CreateTriangleMesh(int W,int H, unsigned subdivide)
{
    std::mt19937 rnd;
    auto random = [&rnd](unsigned size){ return std::uniform_int_distribution<>(0, (size)-1)(rnd); };

    // Create edge vertices
    std::vector<std::array<int,2>> points{ {0,0}, {W-1,0}, {W-1,H-1}, {0,H-1} };
    // Create two triangles covering the entire screen
    std::vector<std::array<int,3>> triangles = { {0,1,2}, {0,2,3} };
    // Subdivide the triangles a few times to create a more interesting mosaic
    for(unsigned n=0; n<subdivide; ++n)
    {
        // Choose the triangle & edge that is longest
        auto len = [&](int e,auto&b) { auto[x0,y0]=points[b[e]]; auto [x1,y1]=points[b[(e+1)%3]];
                                       return (x1-x0)*(x1-x0)+(y1-y0)*(y1-y0); };
        unsigned which = 0;
        int      edge  = 0;
        for(unsigned p=0; p<triangles.size()*3; ++p)
            if(len(p%3, triangles[p/3]) > len(edge, triangles[which]))
                { edge = p%3; which = p/3; }
        auto before = triangles[which], neighbor = before;
        // Choose a random near-mid point along the longest edge
        auto p0 = before[edge%3], p1 = before[(edge+1)%3], p2 = before[(edge+2)%3], q2 = p2;
        int bias = 0x7000, where = random(0x10000-bias*2)+bias, point = points.size();
        // Find the triangle on the opposite side of this edge
        unsigned other;
        for(other = 0; other < triangles.size()*3; ++other)
            if(triangles[other/3][other%3] == p1 && triangles[other/3][(other+1)%3] == p0)
            {
                neighbor  = triangles[other/3];
                q2        = neighbor[(other+2)%3];
                //std::printf("Chose edge %d on %u: %d,%d(,%d)\n", edge,which, p0,p1,p2);
                //std::printf("Neighbor: edge %d on %u (%d)\n", other%3, other/3, q2);
                other /= 3;
                break;
            }
        if(other >= triangles.size()*3)
        {
            // There was no triangle on the opposite side:
            // Create two new triangles. Replace old triangle with one of them, and append the other one.:
            triangles[which] =   { p0, point, p2 };
            triangles.push_back( { p2, point, p1 } );
        }
        else
        {
            // Split both triangles into two.
            //std::printf("Making %d,%d,%d and %d,%d,%d\n", p0,point,p2, p2,point,p1);
            //std::printf("Making %d,%d,%d and %d,%d,%d\n", point,q2,p1, point,p0,q2);
            triangles[which] =   { p0, point, p2 };
            triangles[other] =   { point, q2, p1 };
            triangles.push_back( { p2, point, p1 } );
            triangles.push_back( { point, p0, q2 } );
        }
        points.push_back( { points[p0][0] + (points[p1][0]-points[p0][0])*where/0x10000,
                            points[p0][1] + (points[p1][1]-points[p0][1])*where/0x10000 } );
        //std::printf("Point generated: %d,%d\n", points.back()[0], points.back()[1]);
    }
    std::vector< std::array<std::array<int,2>,3> > result;
    for(auto& t: triangles)
        result.push_back( {points[t[0]],points[t[1]],points[t[2]] });
    //for(auto& t: result)
    //    std::printf("%d %d, %d %d, %d %d\n",
    //        t[0][0],t[0][1], t[1][0],t[1][1], t[2][0],t[2][1]);
    return result;
}
