// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>

extern TraceUI* traceUI;

#include <iostream>
#include <fstream>

using namespace std;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = false;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

Vec3d RayTracer::trace(double x, double y)
{
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) scene->intersectCache.clear();
  ray r(Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY);
  scene->getCamera().rayThrough(x,y,r);
  Vec3d ret = traceRay(r, traceUI->getDepth());
  ret.clamp();
  return ret;
}

Vec3d RayTracer::tracePixel(int i, int j)
{
	Vec3d col(0,0,0);

	if( ! sceneLoaded() ) return col;

	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

    switch(traceUI->m_aa) {
        case 2:
            for(int x_i = -1; x_i <= 1; x_i += 2) {
                for(int y_i = -1; y_i <= 1; y_i += 2) {
                    col += trace(x + x_i * 0.5 / buffer_width, y + y_i * 0.5 / buffer_height);
                }
            }
            col /= 4.0;
            break;
        case 3:
            for(int x_i = -1; x_i <= 1; ++x_i) {
                for(int y_i = -1; y_i <= 1; ++y_i) {
                    col += trace(x + x_i * 0.5 / buffer_width, y + y_i * 0.5 / buffer_height);
                }
            }
            col /= 9.0;
            break;
        case 4:
            for(int x_i = -2; x_i <= 2; ++x_i) {
                for(int y_i = -2; y_i <= 2; ++y_i) {
                    if(x_i != 0 && y_i != 0)
                        col += trace(x + x_i * 0.25 / buffer_width, y + y_i * 0.25 / buffer_height);
                }
            }
            col /= 16.0;
            break;
        default:
            col = trace(x, y);
    }

	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
	return col;
}


// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
Vec3d RayTracer::traceRay(ray& r, int depth)
{
    isect i;
    Vec3d colorC;

    if(depth >= 0 && scene->intersect(r, i)) {
        const Material& m = i.getMaterial();
        colorC = m.shade(scene, r, i);

        // Reflect ray
        if(m.Refl()) {
            Vec3d reflectDir = r.d - 2 * (i.N * r.d) * i.N;
            ray reflection(r.at(i.t), reflectDir, ray::REFLECTION);
            colorC += prod( traceRay(reflection, depth - 1), m.kr(i) );
        }
        // Refract ray
        if(m.Trans()) {
            Vec3d normal = i.N;
            double cosThetaI = normal * r.d;
            double n;
            
            // Check if incident ray is inside or outside of the object
            if(cosThetaI > 0) {
                // inside
                n = 1.0 / m.index(i);
                normal = -i.N;
                cosThetaI = -cosThetaI;
            } else {
                // outside
                n = m.index(i);
            }
            double cosThetaT = 1.0 - (n * n) * (1.0 - cosThetaI * cosThetaI);

            // Check for TIR, then do refraction
            if(cosThetaT >= 0) {
                cosThetaT = sqrt(cosThetaT);
                Vec3d refractDir = (n * cosThetaI - cosThetaT) * normal - n * r.d;
                ray refraction(r.at(i.t), refractDir, ray::REFRACTION);
                colorC += prod( traceRay(refraction, depth - 1), m.kt(i) );
            }
        }
    } else if(haveCubeMap()) {
        colorC = cubemap->getColor(r);
    } else {
        colorC = Vec3d(0.0, 0.0, 0.0);
    }
    return colorC;
}

RayTracer::RayTracer()
	: scene(0), buffer(0), buffer_width(256), buffer_height(256), m_bBufferReady(false)
{}

RayTracer::~RayTracer()
{
	delete scene;
	delete [] buffer;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene( char* fn ) {
	ifstream ifs( fn );
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}
	
	// Strip off filename, leaving only the path:
	string path( fn );
	if( path.find_last_of( "\\/" ) == string::npos ) path = ".";
	else path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
    Parser parser( tokenizer, path );
	try {
		delete scene;
		scene = 0;
		scene = parser.parseScene();
	} 
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	}
	catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	}
	catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}

	if( !sceneLoaded() ) return false;

    scene->buildBvhTree();
	return true;
}

void RayTracer::traceSetup(int w, int h)
{
	if (buffer_width != w || buffer_height != h)
	{
		buffer_width = w;
		buffer_height = h;
		bufferSize = buffer_width * buffer_height * 3;
		delete[] buffer;
		buffer = new unsigned char[bufferSize];
	}
	memset(buffer, 0, w*h*3);
	m_bBufferReady = true;
}

