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

package com.badlogic.gdx.ai.tests.pfa.tests.navmesh.path;

import com.badlogic.gdx.ai.pfa.Connection;
import com.badlogic.gdx.ai.pfa.DefaultGraphPath;
import com.badlogic.gdx.ai.tests.pfa.tests.navmesh.graph.Triangle;
import com.badlogic.gdx.math.Vector3;

/** @author jsjolund */
public class NavMeshGraphPath extends DefaultGraphPath<Connection<Triangle>> {
	/** The start point when generating a point path for this triangle path */
	public Vector3 start;
	/** The end point when generating a point path for this triangle path */
	public Vector3 end;
	/** If the triangle path is empty, the point path will span this triangle */
	public Triangle startTri;

	/** @return Last triangle in the path. */
	public Triangle getEndTriangle () {
		return (getCount() > 0) ? get(getCount() - 1).getToNode() : startTri;
	}
}
