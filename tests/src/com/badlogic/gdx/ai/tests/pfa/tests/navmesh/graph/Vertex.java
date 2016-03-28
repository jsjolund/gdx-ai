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

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.ObjectSet;

/** A vertex in the navigation mesh.
 * @author jsjolund */
public class Vertex extends Vector3 {

	/** The index of this vertex in {@link Mesh#indices} */
	public final short index;
	/** Adjacent vertices in no particular order */
	public final ObjectSet<Triangle> adjacentTriangles = new ObjectSet<Triangle>();
	/** Adjacent triangles in no particular order */
	public final ObjectSet<Vertex> adjacentVertices = new ObjectSet<Vertex>();

	public Vertex (short index) {
		this.index = index;
	}

	public Vertex (short index, Vector3 pos) {
		super(pos);
		this.index = index;
	}

	public Vertex (short index, float x, float y, float z) {
		super(x, y, z);
		this.index = index;
	}
}
