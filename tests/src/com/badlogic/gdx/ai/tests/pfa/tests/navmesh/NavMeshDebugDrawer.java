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

package com.badlogic.gdx.ai.tests.pfa.tests.navmesh;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Edge;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Triangle;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.path.NavMeshGraphPath;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.path.NavMeshPointPath;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Bits;
import com.badlogic.gdx.utils.StringBuilder;

/** @author jsjolund */
public class NavMeshDebugDrawer {

	private final static Matrix4 TMP_M4 = new Matrix4();
	private final static Vector3 TMP_V = new Vector3();
	private final static Quaternion TMP_Q = new Quaternion();
	private final static Color TMP_COLOR1 = new Color();
	private final static Color TMP_COLOR2 = new Color();
	private final static StringBuilder sb = new StringBuilder();

	private NavMeshDebugDrawer () {
	}

	public static boolean triangleIsVisible (Bits visibleLayers, Triangle t) {
		return (visibleLayers == null || visibleLayers.nextSetBit(t.meshPartIndex) != -1);
	}

	public static void drawVertex (MyShapeRenderer shapeRenderer, Vector3 pos, float size, Color color) {
		shapeRenderer.setColor(color);
		float offset = size / 2;
		shapeRenderer.box(pos.x - offset, pos.y - offset, pos.z + offset, size, size, size);
	}

	public static void drawNavMeshTriangles (MyShapeRenderer shapeRenderer, Bits visibleLayers, NavMesh navMesh, Color color) {
		shapeRenderer.set(MyShapeRenderer.ShapeType.Line);
		for (int i = 0; i < navMesh.graph.getNodeCount(); i++) {
			Triangle t = navMesh.graph.getTriangleFromGraph(i);
			if (triangleIsVisible(visibleLayers, t)) {
				drawTriangle(shapeRenderer, t, color, 1);
			}
		}
	}

	public static void drawNavMeshIndices (SpriteBatch spriteBatch, Bits visibleLayers, Camera camera, BitmapFont font,
		NavMesh navMesh) {
		spriteBatch.setProjectionMatrix(camera.combined);
		for (int i = 0; i < navMesh.graph.getNodeCount(); i++) {
			Triangle t = navMesh.graph.getTriangleFromGraph(i);
			if (triangleIsVisible(visibleLayers, t)) {
				TMP_M4.set(camera.view).inv().getRotation(TMP_Q);
				TMP_M4.setToTranslation(t.getCentroid(TMP_V)).rotate(TMP_Q);
				spriteBatch.setTransformMatrix(TMP_M4);
				font.draw(spriteBatch, Integer.toString(t.graphIndex), 0, 0);
			}
		}
	}

	public static void drawTriangle (MyShapeRenderer shapeRenderer, Triangle tri, Color color, float alpha) {
		shapeRenderer.setColor(color.r, color.g, color.b, alpha);
		shapeRenderer.triangle(tri.a, tri.b, tri.c);
	}

	public static void drawPathTriangles (MyShapeRenderer shapeRenderer, Bits visibleLayers, NavMeshGraphPath navMeshGraphPath,
		Color pathTriColor, float pathTriColorAlpha) {
		for (int i = 0; i < navMeshGraphPath.getCount(); i++) {
			Connection<Triangle> conn = navMeshGraphPath.get(i);
			Triangle fromNode = null;
			if (conn instanceof Edge) {
				fromNode = ((Edge)conn).fromNode;
			}
			if (triangleIsVisible(visibleLayers, fromNode)) drawTriangle(shapeRenderer, fromNode, pathTriColor, pathTriColorAlpha);
		}
	}

	public static void drawSharedEdgesInPath (MyShapeRenderer shapeRenderer, Bits visibleLayers, NavMeshGraphPath navMeshGraphPath,
		Color rightVertexColor, Color leftVertexColor, float pathTriColorAlpha) {
		TMP_COLOR1.set(rightVertexColor).a = pathTriColorAlpha;
		TMP_COLOR2.set(leftVertexColor).a = pathTriColorAlpha;
		// Shared triangle edges
		for (Connection<Triangle> connection : navMeshGraphPath) {
			if (connection instanceof Edge) {
				Edge e = (Edge)connection;
				if (triangleIsVisible(visibleLayers, e.fromNode) || triangleIsVisible(visibleLayers, e.toNode))
					shapeRenderer.line(e.rightVertex, e.leftVertex, TMP_COLOR1, TMP_COLOR2);
			}
		}
	}

	public static void drawPathLines (MyShapeRenderer shapeRenderer, NavMeshPointPath navMeshPointPath, Color pathColor) {
		if (navMeshPointPath.getSize() == 0) return;
		shapeRenderer.setColor(pathColor);
		Vector3 q;
		Vector3 p = navMeshPointPath.getVector(navMeshPointPath.getSize() - 1);
		for (int i = navMeshPointPath.getSize() - 1; i >= 0; i--) {
			q = navMeshPointPath.getVector(i);
			shapeRenderer.line(p, q);
			p = q;
		}
	}

	public static void drawPathPoints (MyShapeRenderer shapeRenderer, NavMeshPointPath navMeshPointPath,
		Color crossedEdgePointColor, float crossedEdgePointSize) {
		if (navMeshPointPath.getSize() == 0) return;
		shapeRenderer.setColor(crossedEdgePointColor);
		Vector3 p;
		for (int i = navMeshPointPath.getSize() - 1; i >= 0; i--) {
			p = navMeshPointPath.getVector(i);
			drawVertex(shapeRenderer, p, crossedEdgePointSize, crossedEdgePointColor);
		}
	}

	public static void drawPathPointTriConnections (SpriteBatch spriteBatch, Camera camera, BitmapFont font,
		NavMeshPointPath navMeshPointPath) {

		spriteBatch.setProjectionMatrix(camera.combined);

		for (int i = 0; i < navMeshPointPath.getSize(); i++) {
			int fromIndex = navMeshPointPath.getFromTriangle(i).getIndex();
			int toIndex = navMeshPointPath.getToTriangle(i).getIndex();

			sb.setLength(0);
			sb.append("from: ").append(fromIndex).append("\n");
			Array<Edge> xEdges = navMeshPointPath.getCrossedEdges(i);
			if (xEdges.size > 1) {
				sb.append("touches: ");
				for (int j = 1; j < xEdges.size; j++) {
					sb.append(xEdges.get(j).fromNode.getIndex());
					if (j != xEdges.size - 1) sb.append(", ");
				}
				sb.append("\n");
			}
			sb.append("to: ").append(toIndex).append("\n");
			TMP_M4.set(camera.view).inv().getRotation(TMP_Q);
			TMP_M4.setToTranslation(navMeshPointPath.getVector(i)).rotate(TMP_Q);
			spriteBatch.setTransformMatrix(TMP_M4);
			font.draw(spriteBatch, sb.toString(), 0, 0);
		}
	}

}
