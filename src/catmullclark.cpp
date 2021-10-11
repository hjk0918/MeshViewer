#include "catmullclark.h"
#include <iostream>

void get_face_points(Mesh *mesh, Mesh *previous)
{
    std::vector<Face *> faceList = previous->faces();
    for (std::vector<Face *>::iterator it = faceList.begin(); it != faceList.end(); ++it)
    {
        Face *face = (*it);
        HEdge *he = face->halfEdge();
        Eigen::Vector3f position(0, 0, 0);
        int numEdges = 0;
        do
        {
            position += he->start()->position();
            ++numEdges;
            he = he->next();
        } while (he != face->halfEdge());
        position /= (1.0 * numEdges);

        face->setFacePoint(new Vertex(position));
        mesh->addVertex(face->facePoint());
    }
}

void get_edge_points(Mesh *mesh, Mesh *previous)
{
    std::vector<HEdge *> HEdgeList = previous->edges();
    for (std::vector<HEdge *>::iterator it = HEdgeList.begin(); it != HEdgeList.end(); ++it)
    {
        HEdge *he = (*it);
        if (he->edgePoint())
            continue;

        // doesn't consider boundary edge
        Eigen::Vector3f position(0, 0, 0);
        if (he->sharp() > 0 || !he->twin()) // s>0 || boundary edge
        {
            position = position + he->start()->position() + he->end()->position();
            position = position / 2.0;
        }
        else // common case
        {
            position = position + he->start()->position() + he->end()->position();
            position = position + he->leftFace()->facePoint()->position();
            position = position + he->twin()->leftFace()->facePoint()->position();
            position = position / 4.0;
        }

        he->setEdgePoint(new Vertex(position));
        he->twin()->setEdgePoint(he->edgePoint());
        mesh->addVertex(he->edgePoint());
    }
}

Eigen::Vector3f getAvgFacePos(Vertex *v) // for close mesh only
{
    Eigen::Vector3f position(0, 0, 0);
    int numFaces = 0;
    OneRingHEdge he_ring = OneRingHEdge(v);
    HEdge *he;
    while (he = he_ring.nextHEdge())
    {
        if (he->leftFace())
        {
            position = position + he->leftFace()->facePoint()->position();
            ++numFaces;
        }
    }
    position = position / (1.0 * numFaces);
    return position;
}

Eigen::Vector3f getAvgEdgePos(Vertex *v)
{
    Eigen::Vector3f position(0, 0, 0);
    int numEdges = 0;
    OneRingHEdge he_ring = OneRingHEdge(v);
    HEdge *he;
    while (he = he_ring.nextHEdge())
    {
        position = position + he->edgePoint()->position();
        ++numEdges;
    }
    position = position / (1.0 * numEdges);
    return position;
}

void get_vertex_points(Mesh *mesh, Mesh *previous)
{
    std::vector<Vertex *> vertexList = previous->vertices();
    for (std::vector<Vertex *>::iterator it = vertexList.begin(); it != vertexList.end(); ++it)
    {
        Vertex *v = (*it);

        int valence = 0;
        std::vector<HEdge *> sharpEdgeList;
        OneRingHEdge he_ring = OneRingHEdge(v);
        HEdge *he;
        while (he = he_ring.nextHEdge())
        {
            ++valence;
            if (he->sharp() > 0 || he->twin()->sharp() > 0)
                sharpEdgeList.push_back(he);
        }
        int numSharpEdges = sharpEdgeList.size();

        Eigen::Vector3f position(0, 0, 0);
        if (numSharpEdges <= 1) // smooth rule
        {
            Eigen::Vector3f avgFacePos = getAvgFacePos(v);
            Eigen::Vector3f avgEdgePos = getAvgEdgePos(v);
            position = ((valence - 2) / (1.0 * valence)) * v->position() + (1.0 / (1.0 * valence)) * (avgEdgePos + avgFacePos);
        }
        else if (numSharpEdges = 2) // crease rule
        {
            position = (6.0 * v->position() + sharpEdgeList[0]->end()->position() + sharpEdgeList[1]->end()->position()) / 8.0;
        }
        else // corner rule
        {
            position = v->position();
        }

        v->setVertexPoint(new Vertex(position));
        mesh->addVertex(v->vertexPoint());
    }
}

// problem here
void get_faces(Mesh *mesh, Mesh *previous)
{
    std::vector<Face *> faceList = previous->faces();
    for (std::vector<Face *>::iterator it = faceList.begin(); it != faceList.end(); ++it)
    {
        Face *face = (*it);
        HEdge *he = face->halfEdge();

        // loop thru the face's edges // flipcode
        int num = 0;
        do
        {
            Vertex *v1 = face->facePoint();
            Vertex *v2 = he->edgePoint();
            Vertex *v3 = he->next()->start()->vertexPoint();
            Vertex *v4 = he->next()->edgePoint();

            mesh->addFace(v1, v2, v3, v4);
            he = he->next();
            num++;
        } while (he != face->halfEdge());
    }
    mesh->clear_boundaryHE();

    mesh->computeVertexNormals();
    mesh->setVertexPosDirty(true);
}
