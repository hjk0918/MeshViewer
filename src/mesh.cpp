#include "mesh.h"
#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Sparse>
#include <queue>
#include <chrono>

HEdge::HEdge(bool b)
{
	mBoundary = b;

	mTwin = nullptr;
	mPrev = nullptr;
	mNext = nullptr;

	mStart = nullptr;
	mFace = nullptr;

	mEdgePoint = nullptr;

	mFlag = false;
	mValid = true;

	mSharp = 0;
}

HEdge *HEdge::twin() const
{
	return mTwin;
}

HEdge *HEdge::setTwin(HEdge *e)
{
	mTwin = e;
	return mTwin;
}

HEdge *HEdge::prev() const
{
	return mPrev;
}

HEdge *HEdge::setPrev(HEdge *e)
{
	mPrev = e;
	return mPrev;
}

HEdge *HEdge::next() const
{
	return mNext;
}

HEdge *HEdge::setNext(HEdge *e)
{
	mNext = e;
	return mNext;
}

Vertex *HEdge::start() const
{
	return mStart;
}

Vertex *HEdge::setStart(Vertex *v)
{
	mStart = v;
	return mStart;
}

// new
Vertex *HEdge::edgePoint() const
{
	return mEdgePoint;
}

Vertex *HEdge::setEdgePoint(Vertex *v)
{
	mEdgePoint = v;
	return mEdgePoint;
}

int HEdge::sharp() const
{
	return mSharp;
}

int HEdge::setSharp(int s)
{
	mSharp = s;
	return mSharp;
}

Vertex *HEdge::end() const
{
	return mNext->start();
}

Face *HEdge::leftFace() const
{
	return mFace;
}

Face *HEdge::setFace(Face *f)
{
	mFace = f;
	return mFace;
}

bool HEdge::flag() const
{
	return mFlag;
}

bool HEdge::setFlag(bool b)
{
	mFlag = b;
	return mFlag;
}

bool HEdge::isBoundary() const
{
	return mBoundary;
}

bool HEdge::isValid() const
{
	return mValid;
}

bool HEdge::setValid(bool b)
{
	mValid = b;
	return mValid;
}

OneRingHEdge::OneRingHEdge(const Vertex *v)
{
	if (v == nullptr)
	{
		mStart = nullptr;
		mNext = nullptr;
	}
	else
	{
		mStart = v->halfEdge();
		mNext = v->halfEdge();
	}
}

HEdge *OneRingHEdge::nextHEdge()
{
	HEdge *ret = mNext;
	if (mNext != nullptr && mNext->prev()->twin() != mStart)
	{
		mNext = mNext->prev()->twin();
	}
	else
	{
		mNext = nullptr;
	}
	return ret;
}

OneRingVertex::OneRingVertex(const Vertex *v) : ring(v)
{
}

Vertex *OneRingVertex::nextVertex()
{
	HEdge *he = ring.nextHEdge();
	return he != nullptr ? he->end() : nullptr;
}

Vertex::Vertex() : mHEdge(nullptr), mFlag(0)
{
	mPosition = Eigen::Vector3f::Zero();
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(const Eigen::Vector3f &v) : mPosition(v), mHEdge(nullptr), mFlag(0)
{
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

Vertex::Vertex(float x, float y, float z) : mHEdge(nullptr), mFlag(0)
{
	mPosition = Eigen::Vector3f(x, y, z);
	mColor = VCOLOR_BLUE;
	mNormal = Eigen::Vector3f::Zero();
}

const Eigen::Vector3f &Vertex::position() const
{
	return mPosition;
}

const Eigen::Vector3f &Vertex::setPosition(const Eigen::Vector3f &p)
{
	mPosition = p;
	return mPosition;
}

const Eigen::Vector3f &Vertex::normal() const
{
	return mNormal;
}

const Eigen::Vector3f &Vertex::setNormal(const Eigen::Vector3f &n)
{
	mNormal = n;
	return mNormal;
}

const Eigen::Vector3f &Vertex::color() const
{
	return mColor;
}

const Eigen::Vector3f &Vertex::setColor(const Eigen::Vector3f &c)
{
	mColor = c;
	return mColor;
}

HEdge *Vertex::halfEdge() const
{
	return mHEdge;
}

HEdge *Vertex::setHalfEdge(HEdge *he)
{
	mHEdge = he;
	return mHEdge;
}

// new
Vertex *Vertex::vertexPoint() const
{
	return mVertexPoint;
}
Vertex *Vertex::setVertexPoint(Vertex *v)
{
	mVertexPoint = v;
	return mVertexPoint;
}

int Vertex::index() const
{
	return mIndex;
}

int Vertex::setIndex(int i)
{
	mIndex = i;
	return mIndex;
}

int Vertex::flag() const
{
	return mFlag;
}

int Vertex::setFlag(int f)
{
	mFlag = f;
	return mFlag;
}

bool Vertex::isValid() const
{
	return mValid;
}

bool Vertex::setValid(bool b)
{
	mValid = b;
	return mValid;
}

bool Vertex::isBoundary() const
{
	OneRingHEdge ring(this);
	HEdge *curr = nullptr;
	while (curr = ring.nextHEdge())
	{
		if (curr->isBoundary())
		{
			return true;
		}
	}
	return false;
}

int Vertex::valence() const
{
	int count = 0;
	OneRingVertex ring(this);
	Vertex *curr = nullptr;
	while (curr = ring.nextVertex())
	{
		++count;
	}
	return count;
}

Face::Face() : mHEdge(nullptr), mFacePoint(nullptr), mValid(true)
{
}

HEdge *Face::halfEdge() const
{
	return mHEdge;
}

HEdge *Face::setHalfEdge(HEdge *he)
{
	mHEdge = he;
	return mHEdge;
}

// new
Vertex *Face::facePoint() const
{
	return mFacePoint;
}
Vertex *Face::setFacePoint(Vertex *fp)
{
	mFacePoint = fp;
	return mFacePoint;
}

bool Face::isBoundary() const
{
	HEdge *curr = mHEdge;
	do
	{
		if (curr->twin()->isBoundary())
		{
			return true;
		}
		curr = curr->next();
	} while (curr != mHEdge);
	return false;
}

bool Face::isValid() const
{
	return mValid;
}

bool Face::setValid(bool b)
{
	mValid = b;
	return mValid;
}

Mesh::Mesh()
{
	mVertexPosFlag = true;
	mVertexNormalFlag = true;
	mVertexColorFlag = true;
}

Mesh::~Mesh()
{
	clear();
}

const std::vector<HEdge *> &Mesh::edges() const
{
	return mHEdgeList;
}

const std::vector<HEdge *> &Mesh::boundaryEdges() const
{
	return mBHEdgeList;
}

const std::vector<Vertex *> &Mesh::vertices() const
{
	return mVertexList;
}

const std::vector<Face *> &Mesh::faces() const
{
	return mFaceList;
}

bool Mesh::isVertexPosDirty() const
{
	return mVertexPosFlag;
}

void Mesh::setVertexPosDirty(bool b)
{
	mVertexPosFlag = b;
}

bool Mesh::isVertexNormalDirty() const
{
	return mVertexNormalFlag;
}

void Mesh::setVertexNormalDirty(bool b)
{
	mVertexNormalFlag = b;
}

bool Mesh::isVertexColorDirty() const
{
	return mVertexColorFlag;
}

void Mesh::setVertexColorDirty(bool b)
{
	mVertexColorFlag = b;
}

bool Mesh::loadMeshFile(const std::string filename)
{
	// Use libigl to parse the mesh file
	bool iglFlag = igl::read_triangle_mesh(filename, mVertexMat, mFaceMat);
	// bool iglFlag = igl::readOBJ(filename, mVertexMat, mFaceMat);
	// bool iglFlag = true;
	// std::vector<std::vector<double>> vV, vTC, vN;
	// std::vector<std::vector<int>> vF, vFTC, vFN;

	// if (!igl::readOBJ(filename, vV, vTC, vN, vF, vFTC, vFN))
	// 	return false;

	// if (!igl::list_to_matrix(vV, mVertexMat))
	// 	return false;

	// int numFacePoints = vF[0].size();
	// if (numFacePoints == 3 && !igl::list_to_matrix(vF, mFaceMat))
	// 	return false;
	// else if (numFacePoints == 4 && !igl::list_to_matrix(vF, mFaceMat_quad))
	// 	return false;
	// else
	// 	return false;

	if (iglFlag)
	{
		clear();

		// Construct the half-edge data structure.
		int numVertices = mVertexMat.rows();
		int numFaces = mFaceMat.rows();

		// Fill in the vertex list -> quad doesn't change
		for (int vidx = 0; vidx < numVertices; ++vidx)
		{
			mVertexList.push_back(new Vertex(mVertexMat(vidx, 0),
											 mVertexMat(vidx, 1),
											 mVertexMat(vidx, 2)));
		}
		// Fill in the face list -> tri or quad
		for (int fidx = 0; fidx < numFaces; ++fidx)
		{
			addFace(mFaceMat(fidx, 0), mFaceMat(fidx, 1), mFaceMat(fidx, 2));
			// if (numFacePoints == 3)
			// 	addFace(mFaceMat(fidx, 0), mFaceMat(fidx, 1), mFaceMat(fidx, 2));
			// else if (numFacePoints == 4)
			// 	addFace(mFaceMat_quad(fidx, 0), mFaceMat_quad(fidx, 1), mFaceMat_quad(fidx, 2), mFaceMat_quad(fidx, 3));
			// else
			// 	std::cout << __FUNCTION__ << ": mesh is not triangular or quadraliteral!\n";
		}

		std::vector<HEdge *> hedgeList;
		for (int i = 0; i < mBHEdgeList.size(); ++i)
		{
			if (mBHEdgeList[i]->start())
			{
				hedgeList.push_back(mBHEdgeList[i]);
			}
			// TODO
		}
		mBHEdgeList = hedgeList;

		for (int i = 0; i < mVertexList.size(); ++i)
		{
			mVertexList[i]->adjHEdges.clear();
			mVertexList[i]->setIndex(i);
			mVertexList[i]->setFlag(0);
		}
	}
	else
	{
		std::cout << __FUNCTION__ << ": mesh file loading failed!\n";
	}
	return iglFlag;
}

static void _setPrevNext(HEdge *e1, HEdge *e2)
{
	e1->setNext(e2);
	e2->setPrev(e1);
}

static void _setTwin(HEdge *e1, HEdge *e2)
{
	e1->setTwin(e2);
	e2->setTwin(e1);
}

static void _setFace(Face *f, HEdge *e)
{
	f->setHalfEdge(e);
	e->setFace(f);
}

void Mesh::addFace(int v1, int v2, int v3)
{
	Face *face = new Face();

	HEdge *hedge[3];
	HEdge *bhedge[3]; // Boundary half-edges
	Vertex *vert[3];

	for (int i = 0; i < 3; ++i)
	{
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = mVertexList[v1];
	vert[1] = mVertexList[v2];
	vert[2] = mVertexList[v3];

	// Connect prev-next pointers
	for (int i = 0; i < 3; ++i)
	{
		_setPrevNext(hedge[i], hedge[(i + 1) % 3]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 3]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[2]);
	_setTwin(hedge[2], bhedge[1]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[2]);
	for (int i = 0; i < 3; ++i)
	{
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 3; ++i)
	{
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 3; ++i) // traverse bhedge
	{
		Vertex *start = bhedge[i]->start();
		Vertex *end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j)
		{
			HEdge *curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start)
			{
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr);	  // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 3; ++i)
	{
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

void Mesh::addFace(int v1, int v2, int v3, int v4)
{
	Face *face = new Face();

	HEdge *hedge[4];
	HEdge *bhedge[4]; // Boundary half-edges
	Vertex *vert[4];

	for (int i = 0; i < 4; ++i)
	{
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = mVertexList[v1];
	vert[1] = mVertexList[v2];
	vert[2] = mVertexList[v3];
	vert[3] = mVertexList[v4];

	// Connect prev-next pointers
	for (int i = 0; i < 4; ++i)
	{
		_setPrevNext(hedge[i], hedge[(i + 1) % 4]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 4]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[3]);
	_setTwin(hedge[3], bhedge[1]);
	_setTwin(hedge[2], bhedge[2]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[3]);
	bhedge[3]->setStart(vert[2]);
	for (int i = 0; i < 4; ++i)
	{
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 4; ++i)
	{
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[3]);
	vert[3]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 4; ++i)
	{
		Vertex *start = bhedge[i]->start();
		Vertex *end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j)
		{
			HEdge *curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start)
			{
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr);	  // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 4; ++i)
	{
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

void Mesh::addFace(Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4)
{
	Face *face = new Face();

	HEdge *hedge[4];
	HEdge *bhedge[4]; // Boundary half-edges
	Vertex *vert[4];

	for (int i = 0; i < 4; ++i)
	{
		hedge[i] = new HEdge();
		bhedge[i] = new HEdge(true);
	}
	vert[0] = v1;
	vert[1] = v2;
	vert[2] = v3;
	vert[3] = v4;

	// Connect prev-next pointers
	for (int i = 0; i < 4; ++i)
	{
		_setPrevNext(hedge[i], hedge[(i + 1) % 4]);
		_setPrevNext(bhedge[i], bhedge[(i + 1) % 4]);
	}

	// Connect twin pointers
	_setTwin(hedge[0], bhedge[0]);
	_setTwin(hedge[1], bhedge[3]);
	_setTwin(hedge[3], bhedge[1]);
	_setTwin(hedge[2], bhedge[2]);

	// Connect start pointers for bhedge
	bhedge[0]->setStart(vert[1]);
	bhedge[1]->setStart(vert[0]);
	bhedge[2]->setStart(vert[3]);
	bhedge[3]->setStart(vert[2]);
	for (int i = 0; i < 4; ++i)
	{
		hedge[i]->setStart(vert[i]);
	}

	// Connect start pointers
	// Connect face-hedge pointers
	for (int i = 0; i < 4; ++i)
	{
		vert[i]->setHalfEdge(hedge[i]);
		vert[i]->adjHEdges.push_back(hedge[i]);
		_setFace(face, hedge[i]);
	}
	vert[0]->adjHEdges.push_back(bhedge[1]);
	vert[1]->adjHEdges.push_back(bhedge[0]);
	vert[2]->adjHEdges.push_back(bhedge[3]);
	vert[3]->adjHEdges.push_back(bhedge[2]);

	// Merge boundary if needed
	for (int i = 0; i < 4; ++i)
	{
		Vertex *start = bhedge[i]->start();
		Vertex *end = bhedge[i]->end();

		for (int j = 0; j < end->adjHEdges.size(); ++j)
		{
			HEdge *curr = end->adjHEdges[j];
			if (curr->isBoundary() && curr->end() == start)
			{
				_setPrevNext(bhedge[i]->prev(), curr->next());
				_setPrevNext(curr->prev(), bhedge[i]->next());
				_setTwin(bhedge[i]->twin(), curr->twin());
				bhedge[i]->setStart(nullptr); // Mark as unused
				curr->setStart(nullptr);	  // Mark as unused
				break;
			}
		}
	}

	// Finally add hedges and faces to list
	for (int i = 0; i < 4; ++i)
	{
		mHEdgeList.push_back(hedge[i]);
		mBHEdgeList.push_back(bhedge[i]);
	}
	mFaceList.push_back(face);
}

void Mesh::clear_boundaryHE()
{
	std::vector<HEdge *> hedgeList;
	for (int i = 0; i < mBHEdgeList.size(); ++i)
	{
		if (mBHEdgeList[i]->start())
		{
			hedgeList.push_back(mBHEdgeList[i]);
		}
	}
	mBHEdgeList = hedgeList;

	for (int i = 0; i < mVertexList.size(); ++i)
	{
		mVertexList[i]->adjHEdges.clear();
		mVertexList[i]->setIndex(i);
		mVertexList[i]->setFlag(0);
	}
}

Eigen::Vector3f Mesh::initBboxMin() const
{
	return (mVertexMat.colwise().minCoeff()).transpose();
}

Eigen::Vector3f Mesh::initBboxMax() const
{
	return (mVertexMat.colwise().maxCoeff()).transpose();
}

void Mesh::groupingVertexFlags()
{
	// Init to 255
	for (Vertex *vert : mVertexList)
	{
		if (vert->flag() != 0)
		{
			vert->setFlag(255);
		}
	}
	// Group handles
	int id = 0;
	std::vector<Vertex *> tmpList;
	for (Vertex *vert : mVertexList)
	{
		if (vert->flag() == 255)
		{
			++id;
			vert->setFlag(id);

			// Do search
			tmpList.push_back(vert);
			while (!tmpList.empty())
			{
				Vertex *v = tmpList.back();
				tmpList.pop_back();

				OneRingVertex orv = OneRingVertex(v);
				while (Vertex *v2 = orv.nextVertex())
				{
					if (v2->flag() == 255)
					{
						v2->setFlag(id);
						tmpList.push_back(v2);
					}
				}
			}
		}
	}
}

// new
void Mesh::addVertex(Vertex *vertex)
{
	mVertexList.push_back(vertex);
}

void Mesh::clear()
{
	for (int i = 0; i < mHEdgeList.size(); ++i)
	{
		delete mHEdgeList[i];
	}
	for (int i = 0; i < mBHEdgeList.size(); ++i)
	{
		delete mBHEdgeList[i];
	}
	for (int i = 0; i < mVertexList.size(); ++i)
	{
		delete mVertexList[i];
	}
	for (int i = 0; i < mFaceList.size(); ++i)
	{
		delete mFaceList[i];
	}

	mHEdgeList.clear();
	mBHEdgeList.clear();
	mVertexList.clear();
	mFaceList.clear();
}

std::vector<int> Mesh::collectMeshStats()
{
	int V = 0; // # of vertices
	int E = 0; // # of half-edges
	int F = 0; // # of faces
	int B = 0; // # of boundary loops
	int C = 0; // # of connected components
	int G = 0; // # of genus

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Collect mesh information as listed above.
	/**********************************************/
	V = this->vertices().size();
	E = this->edges().size();
	F = this->faces().size();
	B = this->countBoundaryLoops();
	C = this->countConnectedComponents();
	if (B = 0)
		G = 1 - (V - E / 2 + F) / 2;

	/*====== Programming Assignment 0 ======*/

	std::vector<int> stats;
	stats.push_back(V);
	stats.push_back(E);
	stats.push_back(F);
	stats.push_back(B);
	stats.push_back(C);
	stats.push_back(G);
	return stats;
}

void Mesh::printMeshStats()
{
	std::vector<int> stats = collectMeshStats();
	std::cout << "#side_num             = " << side_num << "\n";
	std::cout << "#vertices             = " << stats[0] << "\n";
	std::cout << "#half_edges           = " << stats[1] << "\n";
	std::cout << "#faces                = " << stats[2] << "\n";
	std::cout << "#boundary_loops       = " << stats[3] << "\n";
	std::cout << "#connected_components = " << stats[4] << "\n";
	std::cout << "#genus                = " << stats[5] << "\n";
	std::cout << "#mVertexPosFlag       = " << mVertexPosFlag << "\n";
	std::cout << "#mVertexNormalFlag    = " << mVertexNormalFlag << "\n";
	std::cout << "#mVertexColorFlag     = " << mVertexColorFlag << "\n\n";

}

int Mesh::countBoundaryLoops()
{
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/**********************************************/

	/*====== Programming Assignment 0 ======*/
	std::vector<HEdge *> BHe = this->boundaryEdges();

	for (std::vector<HEdge *>::iterator it = BHe.begin(); it != BHe.end(); ++it)
		(*it)->setFlag(false);

	for (std::vector<HEdge *>::iterator it = BHe.begin(); it != BHe.end(); ++it)
	{
		if (!(*it)->flag())
		{
			++count;
			HEdge *curr = (*it);
			do
			{
				curr->setFlag(true);
				curr = curr->next();
			} while (curr != (*it));
		}
	}

	return count;
}

int Mesh::countConnectedComponents()
{
	int count = 0;

	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Helper function for Mesh::collectMeshStats()
	/* Count the number of connected components of
	/* the mesh. (Hint: use a stack)
	/**********************************************/

	/*====== Programming Assignment 0 ======*/

	std::vector<HEdge *> BHe = this->edges();

	for (std::vector<HEdge *>::iterator it = BHe.begin(); it != BHe.end(); ++it)
		(*it)->setFlag(false);

	for (std::vector<HEdge *>::iterator it = BHe.begin(); it != BHe.end(); ++it)
	{
		if (!(*it)->flag())
		{
			++count;
			std::queue<HEdge *> queue;
			(*it)->setFlag(true);
			(*it)->twin()->setFlag(true);
			queue.push(*it);
			while (!queue.empty())
			{
				HEdge *he = queue.front();
				queue.pop();
				OneRingHEdge he_ring(he->end());
				HEdge *curr;
				while (curr = he_ring.nextHEdge())
				{
					if (!curr->flag())
					{
						curr->setFlag(true);
						curr->twin()->setFlag(true);
						queue.push(curr);
					}
				}
			}
		}
	}

	return count;
}

void Mesh::computeVertexNormals()
{
	/*====== Programming Assignment 0 ======*/

	/**********************************************/
	/*          Insert your code here.            */
	/**********************************************/
	/*
	/* Compute per-vertex normal using neighboring
	/* facet information. (Hint: remember using a 
	/* weighting scheme. Plus, do you notice any
	/* disadvantages of your weighting scheme?)
	/**********************************************/

	std::vector<Vertex *> vertices = this->vertices();
	int num_vertices = vertices.size();
	for (int i = 0; i < num_vertices; i++)
	{
		Eigen::Vector3f normal(0, 0, 0); // initialize vertex normal

		std::vector<Vertex *> adj_vertices;
		HEdge *he = vertices[i]->halfEdge();
		adj_vertices.push_back(he->twin()->start());
		HEdge *curr = he->prev()->twin();
		while (curr != he)
		{
			adj_vertices.push_back(curr->prev()->start());
			curr = curr->prev()->twin();
		}
		for (int j = 0; j < adj_vertices.size(); j++)
		{
			Vertex *v1 = adj_vertices[j];
			Vertex *v2 = adj_vertices[(j + 1) % adj_vertices.size()];
			normal += (v1->position() - vertices[i]->position()).cross(v2->position() - vertices[i]->position());
		}
		vertices[i]->setNormal(normal / normal.norm());
	}

	/*====== Programming Assignment 0 ======*/

	// Notify mesh shaders
	setVertexNormalDirty(true);
}

float find_weight(HEdge *curr)
{
	if (curr->next()->end() != curr->prev()->start() ||
		curr->twin()->prev()->start() != curr->twin()->next()->end()) // one triangle is not closed
		return 1 / (curr->end()->position() - curr->start()->position()).norm();
	else // both triangles are closed
	{
		Eigen::Vector3f left1 = (curr->start()->position() - curr->prev()->start()->position());
		Eigen::Vector3f left2 = (curr->end()->position() - curr->next()->end()->position());
		Eigen::Vector3f right1 = (curr->start()->position() - curr->twin()->next()->end()->position());
		Eigen::Vector3f right2 = (curr->end()->position() - curr->twin()->prev()->start()->position());
		float cot1 = left1.dot(left2) / left1.cross(left2).norm();
		float cot2 = right1.dot(right2) / right1.cross(right2).norm();
		return 0.5 * (cot1 + cot2);
	}
}

void Mesh::umbrellaSmooth(bool cotangentWeights)
{
	/*====== Programming Assignment 1 ======*/
	auto start = std::chrono::high_resolution_clock::now();
	int num_vertices = mVertexList.size();
	std::vector<Eigen::Vector3f> positions(num_vertices); // initialize vector with size numVertex
	float lambda = 0.5f;								  // shrink rate
	float mu = 0;										  // inflate rate
	bool inflate_flag = false;

	if (cotangentWeights)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 1: Implement the cotangent weighting 
		/* scheme for explicit mesh smoothing. 
		/*
		/* Hint:
		/* It is advised to double type to store the 
		/* weights to avoid numerical issues.
		/**********************************************/

		// without inflation //
		// std::vector<Eigen::Triplet<float>> tripletList;
		// for (int i = 0; i < num_vertices; i++)
		// {
		// 	// find all adjacent vertices //
		// 	Vertex *vertex = mVertexList[i];
		// 	OneRingHEdge he_ring = OneRingHEdge(vertex);
		// 	std::vector<HEdge *> adj_he;
		// 	HEdge *it;
		// 	while (it = he_ring.nextHEdge())
		// 		adj_he.push_back(it);

		// 	// construct sparse matrix A with tripletList //
		// 	float total_weights = 0;
		// 	std::vector<float> weight_list;
		// 	std::vector<int> index_list;
		// 	for (int j = 0; j < adj_he.size(); j++)
		// 	{
		// 		int index = std::find(mVertexList.begin(), mVertexList.end(), adj_he[j]->end()) - mVertexList.begin();
		// 		if (index == num_vertices)
		// 			continue;
		// 		index_list.push_back(index);
		// 		double weight = find_weight(adj_he[j]);
		// 		weight_list.push_back(weight);
		// 		total_weights += weight;
		// 	}

		// 	for (int j = 0; j < index_list.size(); j++)
		// 		tripletList.push_back(Eigen::Triplet<float>(i, index_list[j], -weight_list[j] / total_weights));
		// 	tripletList.push_back(Eigen::Triplet<float>(i, i, 1));
		// }
		// Eigen::SparseMatrix<float> L(num_vertices, num_vertices);
		// L.setFromTriplets(tripletList.begin(), tripletList.end());

		// Eigen::MatrixXf P(num_vertices, 3);
		// for (int i = 0; i < num_vertices; ++i)
		// 	P.row(i) = mVertexList[i]->position().transpose();
		// P = P - lambda * L * P;

		// for (int i = 0; i < num_vertices; ++i)
		// 	mVertexList[i]->setPosition(P.row(i).transpose());

		// with inflation //

		do
		{
			// calculate target position (laplacian vector)
			for (int i = 0; i < num_vertices; i++)
			{
				Eigen::Vector3f position(0, 0, 0);
				double total_weights = 0; // initialize the position

				HEdge *he = mVertexList[i]->halfEdge();
				HEdge *curr = he;

				double weight = find_weight(curr);
				position += curr->twin()->start()->position() * weight;
				total_weights += weight;
				curr = curr->prev()->twin();

				while (curr != he)
				{
					weight = find_weight(curr);
					position += curr->twin()->start()->position() * weight;
					total_weights += weight;
					curr = curr->prev()->twin();
				}

				positions[i] = position / total_weights;
			}
			// determine whether to shrink or inflate
			float co;
			if (!inflate_flag)
				co = lambda;
			else
				co = mu;

			for (int i = 0; i < num_vertices; i++)
				mVertexList[i]->setPosition(co * (positions[i] - mVertexList[i]->position()) + mVertexList[i]->position());

			if (!inflate_flag)
				inflate_flag = true;
			else
				break;
		} while (true);
	}
	else
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the uniform weighting 
		/* scheme for explicit mesh smoothing.
		/**********************************************/

		// without inflation
		// std::vector<Eigen::Triplet<float>> tripletList;
		// for (int i = 0; i < num_vertices; i++)
		// {
		// 	// find all adjacent vertices //
		// 	Vertex *vertex = mVertexList[i];
		// 	OneRingVertex v_ring = OneRingVertex(vertex);
		// 	std::vector<Vertex *> adj_vertices;
		// 	Vertex *it;
		// 	while (it = v_ring.nextVertex())
		// 		adj_vertices.push_back(it);

		// 	// construct sparse matrix A with tripletList //
		// 	tripletList.push_back(Eigen::Triplet<float>(i, i, 1));
		// 	for (int j = 0; j < adj_vertices.size(); j++)
		// 	{
		// 		int index = std::find(mVertexList.begin(), mVertexList.end(), adj_vertices[j]) - mVertexList.begin();
		// 		if (index == num_vertices)
		// 			continue;
		// 		tripletList.push_back(Eigen::Triplet<float>(i, index, (float)(-1.0 / adj_vertices.size())));
		// 	}
		// }
		// Eigen::SparseMatrix<float> L(num_vertices, num_vertices);
		// L.setFromTriplets(tripletList.begin(), tripletList.end());

		// Eigen::MatrixXf P(num_vertices, 3);
		// for (int i = 0; i < num_vertices; ++i)
		// 	P.row(i) = mVertexList[i]->position().transpose();
		// P = P - lambda * L * P;

		// for (int i = 0; i < num_vertices; ++i)
		// 	mVertexList[i]->setPosition(P.row(i).transpose());

		// with inflation //
		do
		{
			// calculate target position (laplacian vector)
			for (int i = 0; i < num_vertices; i++)
			{
				std::vector<Vertex *> adj_vertices; // collect all adjacent vertices
				HEdge *he = mVertexList[i]->halfEdge();
				adj_vertices.push_back(he->twin()->start());
				HEdge *curr = he->prev()->twin();
				while (curr != he)
				{
					adj_vertices.push_back(curr->prev()->start());
					curr = curr->prev()->twin();
				}

				Eigen::Vector3f position(0, 0, 0); // initialize the position
				for (int i = 0; i < adj_vertices.size(); i++)
					position += adj_vertices[i]->position();
				positions[i] = position / adj_vertices.size();
			}
			// determine whether to shrink or inflate
			double co;
			if (!inflate_flag)
				co = lambda;
			else
				co = mu;
			for (int i = 0; i < num_vertices; i++)
				mVertexList[i]->setPosition(co * (positions[i] - mVertexList[i]->position()) + mVertexList[i]->position());
			if (!inflate_flag)
				inflate_flag = true;
			else
				break;
		} while (true);
	}
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("Explicit Smooth, Cot=%d, millisecs=%lld", cotangentWeights, duration.count());
	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}

void Mesh::implicitUmbrellaSmooth(bool cotangentWeights)
{
	/*====== Programming Assignment 1 ======*/
	int num_vertices = mVertexList.size();
	std::vector<Eigen::Vector3f> positions(num_vertices); // initialize vector with size numVertex
	float lambda = 0.5;									  // shrink rate
	float mu = -0.68;									  // inflate rate
	bool inflate_flag = false;

	/* A sparse linear system Ax=b solver using the conjugate gradient method. */
	auto fnConjugateGradient = [](const Eigen::SparseMatrix<float> &A,
								  const Eigen::VectorXf &b,
								  int maxIterations,
								  float errorTolerance,
								  Eigen::VectorXf &x)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Params:
		/*  A: 
		/*  b: 
		/*  maxIterations:	Max number of iterations
		/*  errorTolerance: Error tolerance for the early stopping condition
		/*  x:				Stores the final solution, but should be initialized. 
		/**********************************************/
		/*
		/* Step 1: Implement the biconjugate gradient
		/* method.
		/* Hint: https://en.wikipedia.org/wiki/Biconjugate_gradient_method
		/**********************************************/

		Eigen::BiCGSTAB<Eigen::SparseMatrix<float>> solver;
		solver.setMaxIterations(maxIterations);
		solver.setTolerance(errorTolerance);
		solver.compute(A);
		x = solver.solve(b);
	};

	/* IMPORTANT:
	/* Please refer to the following link about the sparse matrix construction in Eigen. */
	/* http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3 */

	auto start = std::chrono::high_resolution_clock::now();
	Eigen::SparseMatrix<float> L(num_vertices, num_vertices);
	if (cotangentWeights)
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 2: Implement the cotangent weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/*
		/* Hint:
		/* It is advised to double type to store the
		/* weights to avoid numerical issues.
		/**********************************************/

		std::vector<Eigen::Triplet<float>> tripletList;
		for (int i = 0; i < num_vertices; i++)
		{
			// find all adjacent vertices //
			Vertex *vertex = mVertexList[i];
			OneRingHEdge he_ring = OneRingHEdge(vertex);
			std::vector<HEdge *> adj_he;
			HEdge *it;
			while (it = he_ring.nextHEdge())
				adj_he.push_back(it);

			// construct sparse matrix A with tripletList //
			float total_weights = 0;
			std::vector<float> weight_list;
			std::vector<int> index_list;
			for (int j = 0; j < adj_he.size(); j++)
			{
				int index = std::find(mVertexList.begin(), mVertexList.end(), adj_he[j]->end()) - mVertexList.begin();
				if (index == num_vertices)
					continue;
				index_list.push_back(index);
				double weight = find_weight(adj_he[j]);
				weight_list.push_back(weight);
				total_weights += weight;
			}

			for (int j = 0; j < index_list.size(); j++)
				tripletList.push_back(Eigen::Triplet<float>(i, index_list[j], -weight_list[j] / total_weights));
			tripletList.push_back(Eigen::Triplet<float>(i, i, 1));
		}
		L.setFromTriplets(tripletList.begin(), tripletList.end());
	}
	else
	{
		/**********************************************/
		/*          Insert your code here.            */
		/**********************************************/
		/*
		/* Step 3: Implement the uniform weighting 
		/* scheme for implicit mesh smoothing. Use
		/* the above fnConjugateGradient for solving
		/* sparse linear systems.
		/**********************************************/

		std::vector<Eigen::Triplet<float>> tripletList;
		for (int i = 0; i < num_vertices; i++)
		{
			// find all adjacent vertices //
			Vertex *vertex = mVertexList[i];
			OneRingVertex v_ring = OneRingVertex(vertex);
			std::vector<Vertex *> adj_vertices;
			Vertex *it;
			while (it = v_ring.nextVertex())
				adj_vertices.push_back(it);

			// construct sparse matrix A with tripletList //
			tripletList.push_back(Eigen::Triplet<float>(i, i, 1));
			for (int j = 0; j < adj_vertices.size(); j++)
			{
				int index = std::find(mVertexList.begin(), mVertexList.end(), adj_vertices[j]) - mVertexList.begin();
				if (index == num_vertices)
					continue;
				tripletList.push_back(Eigen::Triplet<float>(i, index, (float)(-1.0 / adj_vertices.size())));
			}
		}
		L.setFromTriplets(tripletList.begin(), tripletList.end());
	}
	Eigen::SparseMatrix<float> I(num_vertices, num_vertices);
	I.setIdentity();

	Eigen::SparseMatrix<float> A(num_vertices, num_vertices); // A = (I + lambda * L)
	A = I + lambda * L;

	Eigen::MatrixXf P(num_vertices, 3);
	Eigen::VectorXf P_x(num_vertices), P_y(num_vertices), P_z(num_vertices);
	for (int i = 0; i < num_vertices; ++i)
	{
		P_x(i) = mVertexList[i]->position()(0);
		P_y(i) = mVertexList[i]->position()(1);
		P_z(i) = mVertexList[i]->position()(2);
	}

	Eigen::VectorXf x(P_x), y(P_y), z(P_z);
	int maxIteration = 1000000;
	float errorTolerance = 1.0e-11;
	fnConjugateGradient(A, P_x, maxIteration, errorTolerance, x);
	fnConjugateGradient(A, P_y, maxIteration, errorTolerance, y);
	fnConjugateGradient(A, P_z, maxIteration, errorTolerance, z);

	Eigen::Vector3f position;
	for (int i = 0; i < num_vertices; ++i)
	{
		position(0) = x(i);
		position(1) = y(i);
		position(2) = z(i);
		mVertexList[i]->setPosition(position);
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	printf("Explicit Smooth, Cot=%d, millisecs=%lld", cotangentWeights, duration.count());

	/*====== Programming Assignment 1 ======*/

	computeVertexNormals();
	// Notify mesh shaders
	setVertexPosDirty(true);
}
