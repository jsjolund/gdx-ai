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

import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.StringBuilder;

/** A {@link Triangle} is a node of the {@link NavMeshGraph}.
 *
 * @author jsjolund */
public class Triangle {

	/** Index of this triangle in the path finding graph */
	public final int graphIndex;
	/** Index of the mesh part which contains this triangle */
	public final int meshPartIndex;

	public final Vertex a;
	public final Vertex b;
	public final Vertex c;

	public Edge ab;
	public Edge bc;
	public Edge ca;

	/** @param a A vertex in the triangle
	 * @param b
	 * @param c
	 * @param graphIndex Index of triangle in the model indices array (when vertex indices grouped in three). This is also the
	 *           index of the triangle in the path finding graph.
	 * @param meshPartIndex */
	public Triangle (Vertex a, Vertex b, Vertex c, int graphIndex, int meshPartIndex) {
		this.a = a;
		this.b = b;
		this.c = c;
		this.graphIndex = graphIndex;
		this.meshPartIndex = meshPartIndex;
	}

	public void subdivide () {
		Vertex centroid = new Vertex((short)-1, getCentroid(new Vector3()));
		Vertex abMid = new Vertex((short)-1, new Vector3(a).add(b).scl(0.5f));
		Vertex bcMid = new Vertex((short)-1, new Vector3(b).add(c).scl(0.5f));
		Vertex caMid = new Vertex((short)-1, new Vector3(c).add(a).scl(0.5f));

	}

	public Vector3 getCentroid (Vector3 out) {
		return out.set(a).add(b).add(c).scl(1f / 3f);
	}

	@Override
	public String toString () {
		final StringBuilder sb = new StringBuilder("Triangle{");
		sb.append("graphIndex=").append(graphIndex);
		sb.append(", meshPartIndex=").append(meshPartIndex);
		sb.append('}');
		return sb.toString();
	}

	/** @return Index of triangle in the model indices array (when vertex indices grouped in three). This is also the index of the
	 *         triangle in the path finding graph. */
	public int getIndex () {
		return graphIndex;
	}

	/** Calculates the angle in radians between a reference vector and the (plane) normal of the triangle.
	 *
	 * @param reference
	 * @return */
	public float getAngle (Vector3 reference) {
		float x = reference.x;
		float y = reference.y;
		float z = reference.z;
		reference.set(a).sub(b).crs(b.x - c.x, b.y - c.y, b.z - c.z).nor();
		float angle = (float)Math.acos(reference.dot(x, y, z) / (reference.len() * Math.sqrt(x * x + y * y + z * z)));
		reference.set(x, y, z);
		return angle;
	}

	/** Calculates a random point in this triangle.
	 *
	 * @param out Output vector
	 * @return Output for chaining */
	public Vector3 getRandomPoint (Vector3 out) {
		final float sr1 = (float)Math.sqrt(MathUtils.random());
		final float r2 = MathUtils.random();
		final float k1 = 1 - sr1;
		final float k2 = sr1 * (1 - r2);
		final float k3 = sr1 * r2;
		out.x = k1 * a.x + k2 * b.x + k3 * c.x;
		out.y = k1 * a.y + k2 * b.y + k3 * c.y;
		out.z = k1 * a.z + k2 * b.z + k3 * c.z;
		return out;
	}

	/** Calculates the area of the triangle.
	 *
	 * @return */
	public float area () {
		final float abx = b.x - a.x;
		final float aby = b.y - a.y;
		final float abz = b.z - a.z;
		final float acx = c.x - a.x;
		final float acy = c.y - a.y;
		final float acz = c.z - a.z;
		final float r = aby * acz - abz * acy;
		final float s = abz * acx - abx * acz;
		final float t = abx * acy - aby * acx;
		return 0.5f * (float)Math.sqrt(r * r + s * s + t * t);
	}
}
