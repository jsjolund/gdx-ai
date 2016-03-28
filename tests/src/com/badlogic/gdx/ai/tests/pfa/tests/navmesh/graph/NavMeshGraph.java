/*******************************************************************************
 * Copyright 2015 See AUTHORS file.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

package com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.ai.pfa.indexed.IndexedGraph;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.ObjectSet;

import java.nio.FloatBuffer;

/** Creates an indexed graph from a the {@link Mesh} of a {@link Model}, for use with navigation mesh path finding. Duplicate
 * vertices in the {@link Mesh} (which share the same position) will be merged into one. Only single mesh models are supported.
 * The mesh must not contain any isolated vertices or edges.
 *
 * @author jsjolund */
public class NavMeshGraph implements IndexedGraph<Triangle> {

	private final Array<Connection<Triangle>> TMP_TRI = new Array<Connection<Triangle>>();

	final IntMap<Array<Triangle>> meshPartTriangles = new IntMap<Array<Triangle>>();
	final Array<Triangle> modelTriangles = new Array<Triangle>();
	final int meshPartCount;

	/** See {@link NavMeshGraph}
	 * @param model Model for which to create path finding graph */
	public NavMeshGraph (Model model) {
		Mesh mesh = model.meshes.first();

		meshPartCount = model.meshParts.size;

		// Create vertex object for each vertex in the mesh
		Vertex[] vertices = createVertices(mesh);
		// Merge duplicates with same position, map to vertex index
		IntMap<Vertex> indexToVertexMap = createIndexToVertexMap(vertices);

		// Populate the graph array with triangles
		short[] indices = new short[mesh.getNumIndices()];
		mesh.getIndices(indices);
		int graphIndex = 0;
		for (int meshPartIndex = 0; meshPartIndex < model.meshParts.size; meshPartIndex++) {
			MeshPart meshPart = model.meshParts.get(meshPartIndex);
			for (int index = meshPart.offset; index < meshPart.offset + meshPart.size;) {
				Vertex a = indexToVertexMap.get(indices[index++]);
				Vertex b = indexToVertexMap.get(indices[index++]);
				Vertex c = indexToVertexMap.get(indices[index++]);

				Triangle tri = new Triangle(a, b, c, graphIndex++, meshPartIndex);

				a.adjacentVertices.add(b);
				a.adjacentVertices.add(c);
				b.adjacentVertices.add(a);
				b.adjacentVertices.add(c);
				c.adjacentVertices.add(a);
				c.adjacentVertices.add(b);

				a.adjacentTriangles.add(tri);
				b.adjacentTriangles.add(tri);
				c.adjacentTriangles.add(tri);

				modelTriangles.add(tri);

				if (!meshPartTriangles.containsKey(meshPartIndex)) meshPartTriangles.put(meshPartIndex, new Array<Triangle>());
				meshPartTriangles.get(meshPartIndex).add(tri);
			}
		}

		// Map triangle edges
		ObjectSet<Triangle> tmp = new ObjectSet<Triangle>();
		for (Triangle tri : modelTriangles) {
			if (tri.ab == null) tri.ab = findEdge(tri, tri.a, tri.b, tmp);
			if (tri.bc == null) tri.bc = findEdge(tri, tri.b, tri.c, tmp);
			if (tri.ca == null) tri.ca = findEdge(tri, tri.c, tri.a, tmp);
		}
	}

	/** @param tri Triangle for which to find an edge connecting to another triangle.
	 * @param v Vertex from the triangle
	 * @param u Another vertex
	 * @param tmp Temporary storage
	 * @return An edge for which {@link Edge#fromNode} = tri, {@link Edge#rightVertex} = v, {@link Edge#leftVertex} = u,
	 *         {@link Edge#toNode} = other, otherwise null is returned */
	private static Edge findEdge (Triangle tri, Vertex v, Vertex u, ObjectSet<Triangle> tmp) {
		tmp.clear();
		Edge edge = null;
		ObjectSet<Triangle> intersection = intersect(v.adjacentTriangles, u.adjacentTriangles, tmp);
		if (intersection.size == 2 && intersection.remove(tri)) {
			Triangle adjTri = intersection.first();
			edge = new Edge(tri, adjTri, v, u);
			if (v == adjTri.a && u == adjTri.c) adjTri.ca = new Edge(adjTri, tri, u, v);
			if (v == adjTri.c && u == adjTri.b) adjTri.bc = new Edge(adjTri, tri, u, v);
			if (v == adjTri.b && u == adjTri.a) adjTri.ab = new Edge(adjTri, tri, u, v);
		}
		return edge;
	}

	/** @param a A set of triangles
	 * @param b Another set of triangles
	 * @param out Output
	 * @return Output, the intersection of the two sets */
	private static ObjectSet<Triangle> intersect (ObjectSet<Triangle> a, ObjectSet<Triangle> b, ObjectSet<Triangle> out) {
		for (Triangle tri : a)
			if (b.contains(tri)) out.add(tri);
		return out;
	}

	/** @param vertices The array of vertices for which to remove duplicates
	 * @return A map with key as an index from the indices buffer, value as a vertex which has the same position, but maybe another
	 *         index. */
	private static IntMap<Vertex> createIndexToVertexMap (Vertex[] vertices) {
		IntMap<Vertex> indexReplacementMap = new IntMap<Vertex>();
		for (int i = 0; i < vertices.length; i++) {
			final Vertex vertex = vertices[i];
			if (indexReplacementMap.containsKey(vertex.index)) continue;
			indexReplacementMap.put(vertex.index, vertex);
			for (int j = i + 1; j < vertices.length; j++) {
				final Vertex other = vertices[j];
				if (!indexReplacementMap.containsKey(other.index) && other.equals(vertex)) {
					// vertex.index replaces other.index
					vertices[j] = vertex;
					indexReplacementMap.put(other.index, vertex);
				}
			}
		}
		return indexReplacementMap;
	}

	/** @param mesh The mesh for which to create vertex objects
	 * @return An array with all the vertices in this mesh */
	private static Vertex[] createVertices (Mesh mesh) {
		Vertex[] vertexArray = new Vertex[mesh.getNumIndices()];
		FloatBuffer buffer = mesh.getVerticesBuffer();
		int positionOffset = mesh.getVertexAttributes().findByUsage(VertexAttributes.Usage.Position).offset / 4;
		int vertexSize = mesh.getVertexSize() / 4;
		short[] indices = new short[mesh.getNumIndices()];
		mesh.getIndices(indices);
		for (int i = 0; i < indices.length; i++) {
			short index = indices[i];
			int posIdx = index * vertexSize + positionOffset;
			vertexArray[i] = new Vertex(index, buffer.get(posIdx++), buffer.get(posIdx++), buffer.get(posIdx));
		}
		return vertexArray;
	}

	public Triangle getTriangleFromMeshPart (int meshPartIndex, int triIndexInMeshPart) {
		return meshPartTriangles.get(meshPartIndex).get(triIndexInMeshPart);
	}

	public Triangle getTriangleFromGraph (int graphIndex) {
		return modelTriangles.get(graphIndex);
	}

	public int getMeshPartCount () {
		return meshPartCount;
	}

	public int getTriangleCount (int meshPartIndex) {
		return meshPartTriangles.get(meshPartIndex).size;
	}

	@Override
	public int getIndex (Triangle node) {
		return node.graphIndex;
	}

	@Override
	public int getNodeCount () {
		return modelTriangles.size;
	}

	@Override
	public Array<Connection<Triangle>> getConnections (Triangle fromNode) {
		TMP_TRI.clear();
		if (fromNode.ab != null) TMP_TRI.add(fromNode.ab);
		if (fromNode.bc != null) TMP_TRI.add(fromNode.bc);
		if (fromNode.ca != null) TMP_TRI.add(fromNode.ca);
		return TMP_TRI;
	}
}
