#include <cmath>
#include <float.h>
#include <algorithm>
#include <assert.h>
#include "trimesh.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

using namespace std;

Trimesh::~Trimesh()
{
	for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
		delete *i;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const Vec3d &v )
{
    vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
    materials.push_back( m );
}

void Trimesh::addNormal( const Vec3d &n )
{
    normals.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
    int vcnt = vertices.size();

    if( a >= vcnt || b >= vcnt || c >= vcnt ) return false;

    TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
    newFace->setTransform(this->transform);
    if (!newFace->degen) faces.push_back( newFace );


    // Don't add faces to the scene's object list so we can cull by bounding box
    scene->add(newFace);
    return true;
}

char* Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
    if( !materials.empty() && materials.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of materials.";
    if( !normals.empty() && normals.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of normals.";

    return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	double tmin = 0.0;
	double tmax = 0.0;
	typedef Faces::const_iterator iter;
	bool have_one = false;
	for( iter j = faces.begin(); j != faces.end(); ++j )
	  {
	    isect cur;
	    if( (*j)->intersectLocal( r, cur ) )
	      {
		if( !have_one || (cur.t < i.t) )
		  {
		    i = cur;
		    have_one = true;
		  }
	      }
	  }
	if( !have_one ) i.setT(1000.0);
	return have_one;
}

//bool TrimeshFace::intersect(ray& r, isect& i) const {
//  return intersectLocal(r, i);
//}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{
    const Vec3d& a = parent->vertices[ids[0]];
    const Vec3d& b = parent->vertices[ids[1]];
    const Vec3d& c = parent->vertices[ids[2]];

    // Find normal to triangle ABC's plane; coordinates will be relative to a
    Vec3d normal = (b - a) ^ (c - a);
    normal.normalize();

    // Intersect ray with triangle ABC's supporting plane
    double d = normal * a;

    if(normal * r.d == 0)
        return false;

    double t = (d - (normal * r.p)) / (normal * r.d);

    if(t <= RAY_EPSILON)
        return false;

    Vec3d planarIntersect = r.at(t);

    double alpha = ((c - b) ^ (planarIntersect - b)) * normal;
    double beta  = ((a - c) ^ (planarIntersect - c)) * normal;
    double gamma = ((b - a) ^ (planarIntersect - a)) * normal;

    // Check for intersect
    if (alpha >= 0 && beta >= 0 && gamma >= 0) {
        double areaABC = ((b - a) ^ (c - a)) * normal;

        // finalize barycentric coordinates
        alpha /= areaABC;
        beta  /= areaABC;
        gamma /= areaABC;

        // interpolate normal
        if(!parent->vertNorms) {
            parent->generateNormals();
        }
        Vec3d na = parent->normals[ids[0]];
        Vec3d nb = parent->normals[ids[1]];
        Vec3d nc = parent->normals[ids[2]];

        normal = alpha * na + beta * nb + gamma * nc;
        normal.normalize();

        i.setUVCoordinates(Vec2d(alpha, beta));
        i.setT(t);
        i.setMaterial(getMaterial());
        i.setN(normal);
        i.setObject(this);

        return true;
    }
    return false;
}

void Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
    int cnt = vertices.size();
    normals.resize( cnt );
    int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
    memset( numFaces, 0, sizeof(int)*cnt );
    
    for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
    {
		Vec3d faceNormal = (**fi).getNormal();
        
        for( int i = 0; i < 3; ++i )
        {
            normals[(**fi)[i]] += faceNormal;
            ++numFaces[(**fi)[i]];
        }
    }

    for( int i = 0; i < cnt; ++i )
    {
        if( numFaces[i] )
            normals[i]  /= numFaces[i];
    }

    delete [] numFaces;
    vertNorms = true;
}
