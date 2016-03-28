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

package com.badlogic.gdx.ai.tests.pfa.tests.navmesh.heuristics;

import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Triangle;
import com.badlogic.gdx.math.Vector3;

/** @author jsjolund */
public class ClosestCentroidsHeuristic extends NavMeshHeuristic {

	private final static Vector3 TMP_V1 = new Vector3();
	private final static Vector3 TMP_V2 = new Vector3();

	/** Estimates the distance between two triangles, by calculating the distance between their centroids.
	 *
	 * @param node
	 * @param endNode
	 * @return */
	@Override
	public float estimate (Triangle node, Triangle endNode) {
		super.estimate(node, endNode);
		return node.getCentroid(TMP_V1).dst(endNode.getCentroid(TMP_V2));
	}

}
