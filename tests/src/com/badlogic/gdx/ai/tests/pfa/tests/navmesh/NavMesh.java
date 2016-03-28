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

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.ai.pfa.Heuristic;
import com.badlogic.gdx.ai.pfa.indexed.IndexedAStarPathFinder;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.NavMeshGraph;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Triangle;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.path.NavMeshGraphPath;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.math.Intersector;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.math.collision.Ray;
import com.badlogic.gdx.physics.bullet.collision.btBvhTriangleMeshShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.collision.btTriangleIndexVertexArray;
import com.badlogic.gdx.physics.bullet.collision.btTriangleRaycastCallback;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Bits;
import com.badlogic.gdx.utils.Disposable;
import com.badlogic.gdx.utils.FloatArray;

/** @author jsjolund */
public class NavMesh implements Disposable {

	private static final String TAG = "NavMesh";

	private final static Vector3 TMP_V1 = new Vector3();
	private final static Vector3 TMP_V2 = new Vector3();
	private final static Ray TMP_RAY = new Ray();
	private final static Bits TMP_BITS = new Bits();
	private final static Array<Triangle> TMP_TRI_ARR = new Array<Triangle>();
	private final static FloatArray TMP_FLOAT_ARR = new FloatArray();

	public final NavMeshGraph graph;
	private final btBvhTriangleMeshShape collisionShape;
	private final NavMeshRaycastCallback raycastCallback;
	private final IndexedAStarPathFinder<Triangle> pathFinder;

	public Heuristic<Triangle> heuristic;

	public final static Vector3 UP = new Vector3(Vector3.Y);
	public final static Vector3 DOWN = new Vector3(UP).scl(-1);

	public NavMesh (Model model, Heuristic<Triangle> heuristic) {
		this.heuristic = heuristic;
		btTriangleIndexVertexArray vertexArray = new btTriangleIndexVertexArray(model.meshParts);
		collisionShape = new btBvhTriangleMeshShape(vertexArray, true);
		Vector3 navMeshRayFrom = new Vector3();
		Vector3 navMeshRayTo = new Vector3();
		raycastCallback = new NavMeshRaycastCallback(navMeshRayFrom, navMeshRayTo);
		raycastCallback.setFlags(btTriangleRaycastCallback.EFlags.kF_FilterBackfaces);
		graph = new NavMeshGraph(model);
		pathFinder = new IndexedAStarPathFinder<Triangle>(graph);

	}

	public Heuristic<Triangle> getHeuristic () {
		return heuristic;
	}

	public void setHeuristic (Heuristic<Triangle> heuristic) {
		this.heuristic = heuristic;
	}

	public btCollisionShape getShape () {
		return collisionShape;
	}

	@Override
	public void dispose () {
		collisionShape.dispose();
		raycastCallback.dispose();
	}

	/** Get the triangle which this ray intersects. Returns null if no triangle is intersected.
	 *
	 * @param ray
	 * @param distance
	 * @param allowedMeshParts
	 * @return */
	public Triangle rayTest (Ray ray, float distance, Bits allowedMeshParts) {
		Triangle hitTriangle = null;

		Vector3 rayFrom = TMP_V1;
		Vector3 rayTo = TMP_V2;
		rayFrom.set(ray.origin);
		rayTo.set(ray.direction).scl(distance).add(rayFrom);
		raycastCallback.setHitFraction(1);
		raycastCallback.clearReport();
		raycastCallback.setFrom(rayFrom);
		raycastCallback.setTo(rayTo);
		raycastCallback.setAllowedMeshPartIndices(allowedMeshParts);
		collisionShape.performRaycast(raycastCallback, rayFrom, rayTo);

		if (raycastCallback.triangleIndex != -1) {
			hitTriangle = graph.getTriangleFromMeshPart(raycastCallback.partId, raycastCallback.triangleIndex);
		}
		return hitTriangle;
	}

	/** Calculate a triangle graph path between two triangles which are intersected by the rays.
	 *
	 * @param fromRay
	 * @param toRay
	 * @param allowedMeshParts
	 * @param distance
	 * @param path
	 * @return */
	public boolean getPath (Ray fromRay, Ray toRay, Bits allowedMeshParts, float distance, NavMeshGraphPath path) {

		Triangle fromTri = rayTest(fromRay, distance, allowedMeshParts);
		if (fromTri == null) {
			Gdx.app.debug(TAG, "From triangle not found.");
			return false;
		}
		Vector3 fromPoint = new Vector3();
		Intersector.intersectRayTriangle(fromRay, fromTri.a, fromTri.b, fromTri.c, fromPoint);

		return getPath(fromTri, fromPoint, toRay, allowedMeshParts, distance, path);
	}

	/** Calculate a triangle graph path from a start triangle to the triangle which is intersected by a ray.
	 *
	 * @param fromTri
	 * @param fromPoint
	 * @param toRay
	 * @param allowedMeshParts
	 * @param distance
	 * @param path
	 * @return */
	public boolean getPath (Triangle fromTri, Vector3 fromPoint, Ray toRay, Bits allowedMeshParts, float distance,
		NavMeshGraphPath path) {
		Triangle toTri = rayTest(toRay, distance, allowedMeshParts);
		if (toTri == null) {
			Gdx.app.debug(TAG, "To triangle not found.");
			return false;
		}
		Vector3 toPoint = new Vector3();
		Intersector.intersectRayTriangle(toRay, toTri.a, toTri.b, toTri.c, toPoint);

		return getPath(fromTri, fromPoint, toTri, toPoint, path);
	}

	/** Calculate a triangle graph path between two triangles.
	 *
	 * @param fromTri
	 * @param fromPoint
	 * @param toTri
	 * @param toPoint
	 * @param path
	 * @return */
	public boolean getPath (Triangle fromTri, Vector3 fromPoint, Triangle toTri, Vector3 toPoint, NavMeshGraphPath path) {
		if (pathFinder.searchConnectionPath(fromTri, toTri, heuristic, path)) {
			path.start = new Vector3(fromPoint);
			path.end = new Vector3(toPoint);
			path.startTri = fromTri;
			return true;
		}
		Gdx.app.debug(TAG, "Path not found.");
		return false;
	}

	/** Get a random triangle anywhere on the navigation mesh. The probability distribution is even in world space, as opposed to
	 * triangle index, meaning large triangles will be chosen more often than small ones. */
	public Triangle getRandomTriangle () {
		TMP_BITS.clear();
		for (int i = 0; i < graph.getMeshPartCount(); i++) {
			TMP_BITS.set(i);
		}
		return getRandomTriangle(TMP_BITS);
	}

	/** Get a random triangle on the navigation mesh, on any of the allowed mesh parts. The probability distribution is even in
	 * world space, as opposed to triangle index, meaning large triangles will be chosen more often than small ones.
	 * <p/>
	 * Example usage, to get a random point on the second navigation mesh part: allowedMeshParts.clear(); allowedMeshParts.set(1);
	 * Triangle randomTri = navmesh.getRandomTriangle(allowedMeshParts); Vector3 randomPoint = new Vector3();
	 * randomTri.getRandomPoint(randomPoint);
	 *
	 * @param allowedMeshParts Bits representing allowed mesh part indices.
	 * @return A random triangle. */
	public Triangle getRandomTriangle (Bits allowedMeshParts) {
		TMP_FLOAT_ARR.clear();
		TMP_FLOAT_ARR.ordered = true;
		TMP_TRI_ARR.clear();
		TMP_TRI_ARR.ordered = true;

		// To get a uniform distribution over the triangles in the mesh parts
		// we must take areas of the triangles into account.
		for (int mpIndex = 0; mpIndex < graph.getMeshPartCount(); mpIndex++) {
			if (allowedMeshParts.get(mpIndex)) {
				for (int triIndex = 0; triIndex < graph.getTriangleCount(mpIndex); triIndex++) {
					Triangle tri = graph.getTriangleFromMeshPart(mpIndex, triIndex);
					float integratedArea = 0;
					if (TMP_FLOAT_ARR.size > 0) {
						integratedArea = TMP_FLOAT_ARR.get(TMP_TRI_ARR.size - 1);
					}
					TMP_FLOAT_ARR.add(integratedArea + tri.area());
					TMP_TRI_ARR.add(tri);
				}
			}
		}
		if (TMP_FLOAT_ARR.size == 0) {
			return null;
		}
		float r = MathUtils.random(0f, TMP_FLOAT_ARR.get(TMP_FLOAT_ARR.size - 1));
		int i;
		for (i = 0; i < TMP_FLOAT_ARR.size; i++) {
			if (r <= TMP_FLOAT_ARR.get(i)) {
				break;
			}
		}
		return TMP_TRI_ARR.get(i);
	}

	/** Make a ray test at this point, using a ray spanning from far up in the sky, to far down in the ground.
	 *
	 * @param testPoint The test point
	 * @param out The point of intersection between ray and triangle
	 * @param meshPartIndex Which mesh parts to test.
	 * @return The triangle, or null if ray did not hit any triangles. */
	public Triangle verticalRayTest (Vector3 testPoint, Vector3 out, int meshPartIndex) {
		TMP_BITS.clear();
		TMP_BITS.set(meshPartIndex);
		return verticalRayTest(testPoint, out, TMP_BITS);
	}

	/** Make a ray test at this point, using a ray spanning from far up in the sky, to far down in the ground, along the up axis.
	 *
	 * @param testPoint The test point
	 * @param out The point of intersection between ray and triangle
	 * @param allowedMeshParts Which mesh parts to test. Null if all mesh parts should be tested.
	 * @return The triangle, or null if ray did not hit any triangles. */
	public Triangle verticalRayTest (Vector3 testPoint, Vector3 out, Bits allowedMeshParts) {
		TMP_RAY.set(TMP_V1.set(NavMesh.UP).scl(500).add(testPoint), TMP_V2.set(NavMesh.DOWN));
		Triangle hitTri = rayTest(TMP_RAY, 1000, allowedMeshParts);
		if (hitTri == null) {
			out.set(Float.NaN, Float.NaN, Float.NaN);
			return null;
		} else {
			Intersector.intersectRayTriangle(TMP_RAY, hitTri.a, hitTri.b, hitTri.c, out);
			return hitTri;
		}
	}

}
