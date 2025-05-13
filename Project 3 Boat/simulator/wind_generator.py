import numpy as np
from noise import pnoise2
import matplotlib.pyplot as plt


class WindField:
    def __init__(self, max_speed, scale= 100.0, random_seed=42):
        """
        Initialize the wind field generator.
        
        Parameters:
        - max_speed: float, maximum wind speed (magnitude)
        - random_seed: int, seed for reproducible wind patterns
        """
        self.max_speed = max_speed
        self.random_seed = random_seed
        self.scale = scale  # Scale factor for noise coordinates (larger = smoother patterns)
        self.octaves = 6    # Number of noise octaves (more = more detail)
        self.persistence = 0.5  # Noise persistence
        self.lacunarity = 2.0   # Noise lacunarity
        
        # Different seeds for x and y components to create varied patterns
        self.x_seed = random_seed
        self.y_seed = random_seed + 1  # Different seed for y component
        
    def get_wind(self, coords):
        """
        Returns the wind vector at the given coordinates.
        
        Parameters:
        - coords: tuple or list of (x, y) coordinates
        
        Returns:
        - tuple of (wind_x, wind_y) components
        """
        x, y = coords
        
        # Get noise values for x and y components (range [-1, 1])
        nx = pnoise2(x/self.scale, y/self.scale, 
                    octaves=self.octaves,
                    persistence=self.persistence,
                    lacunarity=self.lacunarity,
                    repeatx=1024, repeaty=1024,
                    base=self.x_seed)
        
        ny = pnoise2(x/self.scale, y/self.scale, 
                    octaves=self.octaves,
                    persistence=self.persistence,
                    lacunarity=self.lacunarity,
                    repeatx=1024, repeaty=1024,
                    base=self.y_seed)
        
        # Scale noise to desired wind speed
        wind_x = nx * self.max_speed
        wind_y = ny * self.max_speed
        
        return (wind_x, wind_y)
    
    def plot_wind_field(self, x_range=(-50, 50), y_range=(-50, 50), grid_step=5, size_mult = 1):
        """
        Plot the wind vector field.
        
        Parameters:
        - wind_field: WindField instance
        - x_range: tuple of (min_x, max_x)
        - y_range: tuple of (min_y, max_y)
        - grid_step: spacing between vectors
        """
        # Create grid
        x = np.arange(x_range[0], x_range[1], grid_step)
        y = np.arange(y_range[0], y_range[1], grid_step)
        X, Y = np.meshgrid(x, y)
        
        # Calculate wind vectors
        U = np.zeros_like(X)
        V = np.zeros_like(Y)
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                wind = self.get_wind((X[i,j], Y[i,j]))
                U[i,j] = wind[0] * size_mult
                V[i,j] = wind[1] * size_mult
        
        # Create figure
        plt.figure(figsize=(10, 10))
        plt.quiver(X, Y, U, V, 
                scale=20,          # Adjust this for better arrow sizing
                scale_units='inches',
                angles='xy',
                color='blue',
                width=0.002)
        
        plt.title('Wind Vector Field\n(Max Speed: {} m/s)'.format(self.max_speed))
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        plt.xlim(x_range)
        plt.ylim(y_range)
        plt.show()

# Example usage
if __name__ == "__main__":
    # Create wind field with max speed 15 m/s
    wind = WindField(max_speed=0.06, scale=100, random_seed=42)
    
    # Plot the wind field
    wind.plot_wind_field(x_range=(-100, 100), y_range=(-100, 100), grid_step=5, size_mult=250)