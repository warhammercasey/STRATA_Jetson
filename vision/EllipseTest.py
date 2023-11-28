import numpy as np
import matplotlib.pyplot as plt

def calculate_distance(point1, point2):
    """Calculate the distance between two points."""
    return np.sqrt(np.sum((np.array(point2) - np.array(point1))**2))

def calculate_semi_perimeter(a, b, c):
    """Calculate the semi-perimeter of a triangle."""
    return (a + b + c) / 2

def find_circle_radius(point1, point2, point3):
    """Find the radius of the circle given three points."""
    # Calculate the distances between the points
    a = calculate_distance(point2, point3)
    b = calculate_distance(point1, point3)
    c = calculate_distance(point1, point2)
    
    # Calculate the semi-perimeter
    p = calculate_semi_perimeter(a, b, c)
    
    # Calculate the circumradius using the formula
    radius = (a * b * c) / (4 * np.sqrt(p * (p - a) * (p - b) * (p - c)))
    
    return radius

def inellipse(A, B, C, t):
    ABC = np.array([A, B, C])

    center = np.mean(ABC, axis=0)

    x = center + (C - center)/2*np.cos(t) + 1/(2*np.sqrt(3))*(B - A)*np.sin(t)
    return x

def turn_radius(A, B, C):
    t_values = np.linspace(0, 2*np.pi, 100)
    ellipse_points = np.array([inellipse(A, B, C, t) for t in t_values])

    idx = np.argmin(np.abs(ellipse_points[:, 0]))
    print(ellipse_points[idx])

    return find_circle_radius(ellipse_points[idx - 1], ellipse_points[idx], ellipse_points[idx + 1])


# Example usage:
A = np.array([0, 0])
B = np.array([-12, 0])
C = np.array([0, 12])

print(turn_radius(A, B, C))
print(find_circle_radius([-2, 2], [0, 0], [2, 2]))

# Plotting the Steiner inellipse
t_values = np.linspace(0, 2*np.pi, 100)
ellipse_points = np.array([inellipse(A, B, C, t) for t in t_values])

#print(ellipse_points)

plt.figure(figsize=(8, 8))
plt.plot(ellipse_points[:, 0], ellipse_points[:, 1], label='Steiner Inellipse')
plt.scatter([A[0], B[0], C[0]], [A[1], B[1], C[1]], color='red', label='Triangle Vertices')
plt.plot([A[0], B[0], C[0], A[0]], [A[1], B[1], C[1], A[1]], color='red')
plt.title('Steiner Inellipse of a Triangle')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.axis('equal')
plt.show()