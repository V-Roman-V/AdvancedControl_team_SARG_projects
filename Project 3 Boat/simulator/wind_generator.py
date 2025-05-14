from enum import Enum
from abc import ABC, abstractmethod
import numpy as np
from noise import pnoise2
import matplotlib.pyplot as plt

class IWindField(ABC):
    """
    Abstract base class for wind field generators.
    All concrete wind generator implementations should inherit from this class.
    """
    
    @abstractmethod
    def get_wind(self, coords: tuple) -> tuple:
        """
        Returns the wind vector at the given coordinates.
        
        Parameters:
        - coords: tuple of (x, y) coordinates
        
        Returns:
        - tuple of (wind_x, wind_y) components
        """
        pass

    def plot_wind_field(self, x_range=(-50, 50), y_range=(-50, 50), grid_step=5, size_mult=1):
        """
        Default plot method for wind field visualization.
        Can be overridden by specific wind generators to customize the plotting style.

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
        plt.quiver(X, Y, U, V, scale=20, scale_units='inches', angles='xy', color='blue', width=0.002)
        plt.title('Wind Vector Field')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        plt.xlim(x_range)
        plt.ylim(y_range)
        plt.show()


class PerlinWindField(IWindField):
    def __init__(self, max_speed, scale=100.0, random_seed=42):
        """
        Initialize the wind field generator.
        
        Parameters:
        - max_speed: float, maximum wind speed (magnitude)
        - random_seed: int, seed for reproducible wind patterns
        """
        self.max_speed = max_speed
        self.scale = scale  # Scale factor for noise coordinates (larger = smoother patterns)
        self.octaves = 6    # Number of noise octaves (more = more detail)
        self.persistence = 0.5  # Noise persistence
        self.lacunarity = 2.0   # Noise lacunarity
        
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

    def plot_wind_field(self, x_range=(-10, 10), y_range=(-10, 10), grid_step=0.5, size_mult=250):
        super().plot_wind_field(x_range=x_range, y_range=y_range, grid_step=grid_step, size_mult=size_mult)


class CosineWaveWind(IWindField):
    def __init__(self, base_speed: float, direction: float, 
                 wavelength: float = 20.0, amplitude: float = 0.5):
        """
        Initialize cosine wave wind field generator.
        
        Parameters:
        - base_speed: float, average wind speed (m/s)
        - direction: float, primary wind direction in degrees (0 = east, 90 = north)
        - wavelength: float, distance between wave peaks (meters)
        - amplitude: float, speed variation amplitude (0-1 fraction of base_speed)
        """
        self.base_speed = base_speed
        self.direction_rad = np.deg2rad(direction)
        self.wavelength = wavelength
        self.amplitude = amplitude
        
        # Precompute direction vector
        self.dir_vector = np.array([np.cos(self.direction_rad), np.sin(self.direction_rad)])
    
    def get_wind(self, coords):
        """
        Returns the wind vector at the given coordinates with cosine speed modulation.
        
        Parameters:
        - coords: tuple or list of (x, y) coordinates
        
        Returns:
        - tuple of (wind_x, wind_y) components
        """
        x, y = coords
        
        # Calculate position along perpendicular axis
        perp_distance = np.dot([x, y], self.dir_vector)
        
        # Calculate speed modulation using cosine function
        speed_mod = 1 + self.amplitude * np.cos(2 * np.pi * perp_distance / self.wavelength)
        current_speed = self.base_speed * speed_mod
        
        # Return wind vector in primary direction
        wind_vector = current_speed * self.dir_vector
        return (wind_vector[0], wind_vector[1])

    def plot_wind_field(self, x_range=(-10, 10), y_range=(-10, 10), grid_step=0.5, size_mult=50):
        super().plot_wind_field(x_range=x_range, y_range=y_range, grid_step=grid_step, size_mult=size_mult)

class ConstantWind(IWindField):
    def __init__(self, speed, direction):
        rad = np.deg2rad(direction)
        self.vector = (speed * np.cos(rad), speed * np.sin(rad))

    def get_wind(self, coords):
        return self.vector

    def plot_wind_field(self, x_range=(-10, 10), y_range=(-10, 10), grid_step=1, size_mult=70):
        super().plot_wind_field(x_range=x_range, y_range=y_range, grid_step=grid_step, size_mult=size_mult)


class WindModelType(Enum):
    PERLIN = "perlin"
    COSINE = "cosine"
    CONSTANT = "constant"

class WindModelFactory:
    @staticmethod
    def create(model_type: WindModelType, config: dict) -> IWindField:
        """
        Factory method to create different wind field models based on the type.
        """
        if model_type == WindModelType.PERLIN:
            return PerlinWindField(**config)
        if model_type == WindModelType.COSINE:
            return CosineWaveWind(**config)
        if model_type == WindModelType.CONSTANT:
            return ConstantWind(**config)
        raise ValueError(f"Unknown wind model: {model_type}")


class WindModel:
    @staticmethod
    def create(model_type: str, **kwargs) -> IWindField:
        """
        Creates a wind field model based on a string identifier and parameters.
        """
        try:
            model_type = WindModelType(model_type.lower())
        except ValueError:
            raise ValueError(f"Invalid model type. Available options: {[e.value for e in WindModelType]}")
        
        # Configuration for each model type
        required_params = {
            'perlin': ['max_speed'],
            'cosine': ['base_speed', 'direction'],
            'constant': ['speed', 'direction']
        }[model_type.value]

        missing = [p for p in required_params if p not in kwargs]
        if missing:
            raise ValueError(f"Missing: {missing}")
        return WindModelFactory.create(model_type, kwargs)

# Example
if __name__ == '__main__':
    # Create wind field models using the factory pattern
    wind1 = WindModel.create('cosine', base_speed=0.08, direction=45, wavelength=0.07, amplitude=0.6)
    wind2 = WindModel.create('perlin', max_speed=0.1, scale=5)
    wind3 = WindModel.create('constant', speed=0.1, direction=-45)
    
    # Plot wind fields
    wind1.plot_wind_field(x_range=(-10, 10), y_range=(-10, 10), grid_step=0.5, size_mult=50)
    wind2.plot_wind_field(x_range=(-10, 10), y_range=(-10, 10), grid_step=0.5, size_mult=250)
    wind3.plot_wind_field(x_range=(-10, 10), y_range=(-10, 10), grid_step=1, size_mult=70)
