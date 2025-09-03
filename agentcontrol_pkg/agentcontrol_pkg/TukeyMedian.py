import numpy as np
import random




class TukeyContour:
    """
    Calculates the Tukey depth contour (median region) for a set of 2D points.

    """
    def __init__(self, input_points: np.ndarray, verbose: bool = False):
        self.primal_points = np.asarray(input_points)
        self.verbose = verbose
        self.median_contour = []

        if self.primal_points.shape[0] < 3:
            # Not enough points to form a contour
            return

       
        self._calculate_contour()

    def _cross_product(self, p1, p2, p3):
        """Calculates the 2D cross product to determine orientation."""
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

    def _monotone_chain_convex_hull(self, points: np.ndarray):
        """Computes the convex hull of a set of 2D points."""
        points = sorted(points, key=lambda p: (p[0], p[1]))
        if len(points) <= 2:
            return points

        upper_hull, lower_hull = [], []
        for p in points:
            while len(lower_hull) >= 2 and self._cross_product(lower_hull[-2], lower_hull[-1], p) <= 0:
                lower_hull.pop()
            lower_hull.append(p)

        for p in reversed(points):
            while len(upper_hull) >= 2 and self._cross_product(upper_hull[-2], upper_hull[-1], p) <= 0:
                upper_hull.pop()
            upper_hull.append(p)

        return lower_hull[:-1] + upper_hull[:-1]

    def _calculate_contour(self):
        """Main logic to compute the Tukey median contour."""
        # 1. Duality Transform: Point (px, py) -> Line y = px*x - py
        # We store lines as (m, c) for y = mx + c
        dual_lines = np.array([[p[0], -p[1]] for p in self.primal_points])

        # 2. Find all intersection points of dual lines
        dual_intersections = []
        epsilon = 1e-9
        for i in range(len(dual_lines)):
            for j in range(i + 1, len(dual_lines)):
                m1, c1 = dual_lines[i]
                m2, c2 = dual_lines[j]
                if abs(m1 - m2) > epsilon:
                    x = (c2 - c1) / (m1 - m2)
                    y = m1 * x + c1
                    dual_intersections.append((x, y))

        # 3. Calculate the depth of each intersection point
        max_depth = 0
        intersections_with_depth = []
        for p in dual_intersections:
            px, py = p
            lines_above = np.sum((dual_lines[:, 0] * px + dual_lines[:, 1]) > py + epsilon)
            lines_below = np.sum((dual_lines[:, 0] * px + dual_lines[:, 1]) < py - epsilon)
            depth = min(lines_above, lines_below) + 1 # Depth is 1-indexed
            intersections_with_depth.append({'point': p, 'depth': depth})
            if depth > max_depth:
                max_depth = depth

        if self.verbose:
            print(f"Calculated depths. Maximum depth (k*) is {max_depth}.")

        # 4. Iteratively find a non-empty contour, starting from max_depth
        final_contour_points = []
        k = max_depth
        while k > 0 and not final_contour_points:
            median_dual_vertices = [item['point'] for item in intersections_with_depth if item['depth'] >= k]
            
            if len(median_dual_vertices) < 3:
                k -= 1
                continue

            # 5. Get the convex hull of the k-level region in dual space
            dual_contour_hull = self._monotone_chain_convex_hull(median_dual_vertices)

            # 6. Transform dual hull vertices back to primal lines
            primal_contour_lines = np.array([[p[0], -p[1]] for p in dual_contour_hull])

            # 7. Find intersections of these primal lines
            primal_vertices = []
            for i in range(len(primal_contour_lines)):
                for j in range(i + 1, len(primal_contour_lines)):
                    m1, c1 = primal_contour_lines[i]
                    m2, c2 = primal_contour_lines[j]
                    if abs(m1 - m2) > epsilon:
                        x = (c2 - c1) / (m1 - m2)
                        y = m1 * x + c1
                        primal_vertices.append((x, y))
            
            # 8. The final contour is the convex hull of these primal intersections
            if primal_vertices:
                final_contour_points = self._monotone_chain_convex_hull(primal_vertices)
            
            if not final_contour_points:
                if self.verbose:
                    print(f"Contour with depth {k} is empty, trying depth {k-1}")
                k -= 1

        self.median_contour = np.array(final_contour_points)


