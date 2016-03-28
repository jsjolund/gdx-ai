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

/** @author jsjolund */
public class ClosestVerticesHeuristic extends NavMeshHeuristic {

	/** Estimates the distance between two Triangles, by calculating the distance between their closest vertices.
	 *
	 * @param node
	 * @param endNode
	 * @return */
	@Override
	public float estimate (Triangle node, Triangle endNode) {
		super.estimate(node, endNode);

		float dst2;
		float minDst2 = Float.POSITIVE_INFINITY;
		if ((dst2 = node.a.dst2(endNode.a)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.a.dst2(endNode.b)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.a.dst2(endNode.c)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.b.dst2(endNode.a)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.b.dst2(endNode.b)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.b.dst2(endNode.c)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.c.dst2(endNode.a)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.c.dst2(endNode.b)) < minDst2) minDst2 = dst2;
		if ((dst2 = node.c.dst2(endNode.c)) < minDst2) minDst2 = dst2;
		return (float)Math.sqrt(minDst2);
	}

}
