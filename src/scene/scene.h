//
// scene.h
//
// The Scene class and the geometric types that it can contain.
//

#pragma warning (disable: 4786)


#ifndef __SCENE_H__
#define __SCENE_H__

#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include <memory>

#include "ray.h"
#include "material.h"
#include "camera.h"
#include "bbox.h"

#include "../vecmath/vec.h"
#include "../vecmath/mat.h"

class Light;
class Scene;

template <typename Obj>
class BvhTree;

class SceneElement {

public:
  virtual ~SceneElement() {}

  Scene *getScene() const { return scene; }

  // For debugging purposes, draws using OpenGL
  virtual void glDraw(int quality, bool actualMaterials, bool actualTextures) const  { }

 protected:
 SceneElement( Scene *s )
   : scene( s ) {}

  Scene *scene;
};

class TransformNode {

protected:

  // information about this node's transformation
  Mat4d    xform;
  Mat4d    inverse;
  Mat3d    normi;

  // information about parent & children
  TransformNode *parent;
  std::vector<TransformNode*> children;
    
 public:
  typedef std::vector<TransformNode*>::iterator          child_iter;
  typedef std::vector<TransformNode*>::const_iterator    child_citer;

  ~TransformNode() {
      for(child_iter c = children.begin(); c != children.end(); ++c ) delete (*c);
    }

  TransformNode *createChild(const Mat4d& xform) {
    TransformNode *child = new TransformNode(this, xform);
    children.push_back(child);
    return child;
  }
    
  // Coordinate-Space transformation
  Vec3d globalToLocalCoords(const Vec3d &v) { return inverse * v; }

  Vec3d localToGlobalCoords(const Vec3d &v) { return xform * v; }

  Vec4d localToGlobalCoords(const Vec4d &v) { return xform * v; }

  Vec3d localToGlobalCoordsNormal(const Vec3d &v) {
    Vec3d ret = normi * v;
    ret.normalize();
    return ret;
  }

  const Mat4d& transform() const		{ return xform; }

protected:
  // protected so that users can't directly construct one of these...
  // force them to use the createChild() method.  Note that they CAN
  // directly create a TransformRoot object.
 TransformNode(TransformNode *parent, const Mat4d& xform ) : children() {
      this->parent = parent;
      if (parent == NULL) this->xform = xform;
      else this->xform = parent->xform * xform;  
      inverse = this->xform.inverse();
      normi = this->xform.upper33().inverse().transpose();
    }
};

class TransformRoot : public TransformNode {
 public:
 TransformRoot() : TransformNode(NULL, Mat4d()) {}
};

// A Geometry object is anything that has extent in three dimensions.
// It may not be an actual visible scene object.  For example, hierarchical
// spatial subdivision could be expressed in terms of Geometry instances.
class Geometry : public SceneElement {

protected:
  // intersections performed in the object's local coordinate space
  // do not call directly - this should only be called by intersect()
  virtual bool intersectLocal(ray& r, isect& i ) const = 0;

public:
  // intersections performed in the global coordinate space.
  bool intersect(ray& r, isect& i) const;

  virtual bool hasBoundingBoxCapability() const;
  const BoundingBox& getBoundingBox() const { return bounds; }
  Vec3d getNormal() { return Vec3d(1.0, 0.0, 0.0); }

  virtual void ComputeBoundingBox() {
    // take the object's local bounding box, transform all 8 points on it,
    // and use those to find a new bounding box.

    BoundingBox localBounds = ComputeLocalBoundingBox();
        
    Vec3d min = localBounds.getMin();
    Vec3d max = localBounds.getMax();

    Vec4d v, newMax, newMin;

    v = transform->localToGlobalCoords( Vec4d(min[0], min[1], min[2], 1) );
    newMax = v;
    newMin = v;
    v = transform->localToGlobalCoords( Vec4d(max[0], min[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], max[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], max[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], min[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], min[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], max[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], max[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
		
    bounds.setMax(Vec3d(newMax));
    bounds.setMin(Vec3d(newMin));
  }

  // default method for ComputeLocalBoundingBox returns a bogus bounding box;
  // this should be overridden if hasBoundingBoxCapability() is true.
  virtual BoundingBox ComputeLocalBoundingBox() { return BoundingBox(); }

  void setTransform(TransformNode *transform) { this->transform = transform; };
    
 Geometry(Scene *scene) : SceneElement( scene ) {}

  // For debugging purposes, draws using OpenGL
  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

  // The defult does nothing; this is here because it is not required
  // that you implement this function if you create your own scene objects.
  virtual void glDrawLocal(int quality, bool actualMaterials, bool actualTextures) const { }

 protected:
  BoundingBox bounds;
  TransformNode *transform;
};

// A SceneObject is a real actual thing that we want to model in the 
// world.  It has extent (its Geometry heritage) and surface properties
// (its material binding).  The decision of how to store that material
// is left up to the subclass.
class SceneObject : public Geometry {

 public:
  virtual const Material& getMaterial() const = 0;
  virtual void setMaterial(Material *m) = 0;

  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

 protected:
 SceneObject( Scene *scene )
   : Geometry( scene ) {}
};

// A simple extension of SceneObject that adds an instance of Material
// for simple material bindings.
class MaterialSceneObject : public SceneObject {

public:
  virtual ~MaterialSceneObject() { delete material; }

  virtual const Material& getMaterial() const { return *material; }
  virtual void setMaterial(Material* m)	{ delete material; material = m; }

protected:
 MaterialSceneObject(Scene *scene, Material *mat) 
   : SceneObject(scene), material(mat) {}

  Material* material;
};

// TODO
template <typename Obj>
class BvhTree {

public:

    BvhTree(const std::vector<Obj*> &objects, const BoundingBox &sceneBB) {
        root = new BvhNode(objects, sceneBB, 0);
    }

    bool intersect(ray &r, isect &i) {
        return root->intersect(r, i);
    }

private:
    
    static const int MAX_DEPTH = 15;
    static const int MIN_SIZE  = 3;

    class BvhNode {

    public:

        BvhNode(const std::vector<Obj*> &objects, const BoundingBox &bb, int depth) {
            this->bb = bb;

            if(depth < MAX_DEPTH && objects.size() > MIN_SIZE) {
                // find split dimension
                Vec3d lengths = bb.getMax() - bb.getMin();

                // split on x
                if(lengths[0] > lengths[1] && lengths[0] > lengths[2]) {
                    lengths[1] = 0;
                    lengths[2] = 0;
                }
                // split on y
                else if(lengths[1] > lengths[0] && lengths[1] > lengths[2]) {
                    lengths[0] = 0;
                    lengths[2] = 0;
                }
                // split on z
                else {
                    lengths[0] = 0;
                    lengths[1] = 0;
                }
                double lRatio = 0.5;
                double rRatio = 0.5;

                // perform split
                BoundingBox lBound(bb.getMin(), bb.getMax() - lengths * rRatio);
                BoundingBox rBound(bb.getMin() + lengths * lRatio, bb.getMax());

                // place objects into child bins
                std::vector<Obj*> lObjects;
                std::vector<Obj*> rObjects;

                for(Obj* obj : objects) {
                    if(obj->hasBoundingBoxCapability()) {

                        if(obj->getBoundingBox().intersects(lBound)) {
                            lObjects.push_back(obj);
                        }
                        if(obj->getBoundingBox().intersects(rBound)) {
                            rObjects.push_back(obj);
                        }
                    }
                }
                lChild = new BvhNode(lObjects, lBound, depth + 1);
                rChild = new BvhNode(rObjects, rBound, depth + 1);
            } 
            else {
                this->objects = objects;
            }
        }

        bool intersect(ray &r, isect &i) {
            double tMin;
            double tMax;

            if(!bb.intersect(r, tMin, tMax)) {
                return false;
            }
            // recur if this is not a leaf; BEWARE SHORT CIRCUIT EVALUATION
            else if(lChild != NULL && rChild != NULL) {
                bool have_left  = lChild->intersect(r, i);
                bool have_right = rChild->intersect(r, i);
                return have_left || have_right;
            }
            // If this is a leaf, test all objects
            else {
                bool have_one = false;

                for(Obj* obj : objects) {
                    isect ii;
                    if( obj->intersect(r, ii) && (i.t < RAY_EPSILON || ii.t < i.t) ) {
                        have_one = true;
                        i = ii;
                    }
                }
                return have_one;
            }
        }

    private:
        
        BoundingBox bb;
        std::vector<Obj*> objects;

        BvhNode *lChild = 0;
        BvhNode *rChild = 0;
    };

    BvhNode *root;
};

class Scene {

public:
  typedef std::vector<Light*>::iterator	liter;
  typedef std::vector<Light*>::const_iterator cliter;
  typedef std::vector<Geometry*>::iterator giter;
  typedef std::vector<Geometry*>::const_iterator cgiter;

  TransformRoot transformRoot;

  Scene() : transformRoot(), objects(), lights() {}
  virtual ~Scene();

  void add( Geometry* obj ) {
    obj->ComputeBoundingBox();
	sceneBounds.merge(obj->getBoundingBox());
    objects.push_back(obj);
  }
  void add(Light* light) { lights.push_back(light); }

  bool intersect(ray& r, isect& i) const;

  std::vector<Light*>::const_iterator beginLights() const { return lights.begin(); }
  std::vector<Light*>::const_iterator endLights() const { return lights.end(); }

  std::vector<Geometry*>::const_iterator beginObjects() const { return objects.begin(); }
  std::vector<Geometry*>::const_iterator endObjects() const { return objects.end(); }
        
  const Camera& getCamera() const { return camera; }
  Camera& getCamera() { return camera; }

  // For efficiency reasons, we'll store texture maps in a cache
  // in the Scene.  This makes sure they get deleted when the scene
  // is destroyed.
  TextureMap* getTexture( string name );

  // These two functions are for handling ambient light; in the Phong model,
  // the "ambient" light is considered a property of the _scene_ as a whole
  // and hence should be set here.
  Vec3d ambient() const	{ return ambientIntensity; }
  void addAmbient( const Vec3d& ambient ) { ambientIntensity += ambient; }

  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

  const BoundingBox& bounds() const { return sceneBounds; }

  void buildBvhTree();

 private:
  std::vector<Geometry*> objects;
  std::vector<Geometry*> nonboundedobjects;
  std::vector<Geometry*> boundedobjects;
  std::vector<Light*> lights;
  Camera camera;

  // This is the total amount of ambient light in the scene
  // (used as the I_a in the Phong shading model)
  Vec3d ambientIntensity;

  typedef std::map< std::string, TextureMap* > tmap;
  tmap textureCache;
	
  // Each object in the scene, provided that it has hasBoundingBoxCapability(),
  // must fall within this bounding box.  Objects that don't have hasBoundingBoxCapability()
  // are exempt from this requirement.
  BoundingBox sceneBounds;
  
  BvhTree<Geometry>* bvh;

 public:
  // This is used for debugging purposes only.
  mutable std::vector<std::pair<ray*, isect*> > intersectCache;
};

#endif // __SCENE_H__
