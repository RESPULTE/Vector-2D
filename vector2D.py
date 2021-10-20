from __future__ import annotations
from typing import List, Tuple, Union, Optional, TypeVar, Iterable
from math   import sin, cos, radians, sqrt, ceil, floor, atan2, acos


Number = TypeVar('Number', bound=float)
Coordinate = Union[Tuple[Number, Number], List[Number]]


class Vector2D:


    def __init__(self, *vector: Union[Coordinate, Vector2D]):
        try:
            vector = vector if len(vector) > 1 else vector[0]
            self.x, self.y = float(vector[0]), float(vector[1])
        except (ValueError, TypeError, IndexError) as err:
            raise ValueError(f"'{self.__class__.__name__}' accepts 2 arguements, 'X' and 'Y'") from err

        self._length  = None
        self._changed = False


    @property
    def x(self) -> Number:
        return self._x


    @property
    def y(self) -> Number:
        return self._y


    @property
    def length(self) -> Number:
        if self._length == None or self._changed:
            self._changed = False
            self._length  = round(sqrt((self.x * self.x) + (self.y * self.y)), 2)
        return self._length


    @property
    def astuple(self) -> Coordinate:
        return tuple((self.x, self.y))


    @property
    def asdict(self) -> Dict[str, Number]:
        return {'x': self.x, 'y': self.y}

    
    @x.setter
    def x(self, val: Number):
        if not isinstance(val, (int, float)):
            raise ValueError(f"must be 'int' or 'float', not '{type(val).__name__}'")
        self._x = float(f"{val:2f}")
        self._changed = True


    @y.setter
    def y(self, val: Number):
        if not isinstance(val, (int, float)):
            raise ValueError(f"must be 'int' or 'float', not '{type(val).__name__}'")
        self._y = float(f"{val:2f}")
        self._changed = True


    @classmethod
    def convert(cls, points: List[Tuple[float, float]]) -> List[Vector2D]:
        return [cls(*point) for point in points]


    def scale_to_length(self, length: Number) -> None:
        if self.x != 0 and self.y != 0:
            ratio = length / self.length
            self.x *= ratio
            self.y *= ratio


    def normalize(self) -> Vector2D:
        if self.x != 0 and self.y != 0:
            return Vector2D(self.x / self.length, self.y / self.length)
        return Vector2D(0,0)


    def normalize_ip(self) -> None:
        if self.x != 0 and self.y != 0:
            self.x /= self.length
            self.y /= self.length


    def cross(self, point2: Union[Coordinate, Vector2D], origin: Union[Coordinate, Vector2D] = (0, 0)) -> Number:
        # if point 2 'above' point1 -> positive(+)
        # if point 2 'below' point1 -> negative(-)
        x1, y1 = self - origin
        x2, y2 = point2 - origin
        return (x1 * y2) - (y1 * x2)


    def dot(self, vector: Union[Coordinate, Vector2D]) -> float:
        return (self.x * vector[0]) + (self.y * vector[1])


    def get_midpoint(self, point2: Union[Coordinate, Vector2D]) -> Union[Coordinate, Vector2D]:
        return Vector2D((point1.x + point2.x) / 2, (point1.y + point2.y) / 2)


    def get_perpendicular(self, 
        ref_point: Union[Coordinate, Vector2D]
        ) -> Union[Coordinate, Vector2D]:

        perpendicular_line = Vector2D(self.y, -self.x)
        if perpendicular_line.dot(ref_point) > 0:
            perpendicular_line *= -1
        return perpendicular_line


    def is_perpendicular(self, point2: Union[Coordinate, Vector2D]) -> bool:
        return True if self.dot(point2) == 0 else False 


    def rotate(self, 
        angle: Number, 
        origin: Union[Coordinate, Vector2D] = (0,0),
        clockwise: Optional[bool] = True
        ) -> Union[Coordinate, Vector2D]:
        # formula:
        #  let r = length of the vector
        #  rotated_x = r[cos(original_angle + new_angle)]
        #            = r * [cos(original_angle)cos(new_angle) - sin(original_angle)sin(new_angle)]
        #            = (original_x)(cos(new_angle)) - (original_y)(sin(new_angle))
        #
        #  rotated_y = r[sin(original_angle + new_angle)]
        #            = r * [sin(original_angle)cos(new_angle) + cos(originl_angle)sin(new_angle)]
        #            = (original_y)(cos(new_angle)) + (original_x)(sin(new_angle))
        #
        # translated -> original (in this case)
        angle  = radians(angle)
        cosine, sine = cos(angle), sin(angle)
        translated_x = self.x - origin[0]
        translated_y = self.y - origin[1]

        dx = origin[0] + (translated_x * cosine - translated_y * sine)
        dy = origin[1] + (translated_y * cosine + translated_x * sine)

        return Vector2D(dx, dy) * -1 if clockwise else Vector2D(dx, dy)


    def rotate_ip(self, 
        angle: Number, 
        origin: Optional[Union[Coordinate, Vector2D]] = (0,0),
        clockwise: Optional[bool] = True
        ) -> None:
        # formula:
        #  let r = length of the vector
        #  rotated_x = r[cos(original_angle + new_angle)]
        #            = r * [cos(original_angle)cos(new_angle) - sin(original_angle)sin(new_angle)]
        #            = (original_x)(cos(new_angle)) - (original_y)(sin(new_angle))
        #
        #  rotated_y = r[sin(original_angle + new_angle)]
        #            = r * [sin(original_angle)cos(new_angle) + cos(originl_angle)sin(new_angle)]
        #            = (original_y)(cos(new_angle)) + (original_x)(sin(new_angle))
        #
        # translated -> original (in this case)
        angle        = radians(angle) * -1 if clockwise else radians(angle)
        cosine, sine = cos(angle), sin(angle)
        translated_x = self.x - origin[0]
        translated_y = self.y - origin[1]

        dx = origin[0] + (translated_x * cosine - translated_y * sine)
        dy = origin[1] + (translated_y * cosine + translated_x * sine)

        self.x, self.y = dx, dy


    def angle_to(self, 
        point2   : Vector2D, 
        origin   : Optional[Union[Coordinate, Vector2D]] = (0,0), 
        decimal  : Optional[int] = 1,
        clockwise: Optional[bool] = False,
        vecChange: Optional[bool] = False
        ) -> Number:

        origin_to_point1 = self   - origin
        origin_to_point2 = point2 - origin

        scalar  = origin_to_point1.dot(origin_to_point2)
        length1 = origin_to_point1.length
        length2 = origin_to_point2.length

        angle = round(acos(scalar / (length1 * length2)) * 180 / 3.142, decimal)   

        return angle * -1 if clockwise else angle


    def in_polygon(self, polygon: List[Union[Coordinate, Vector2D]]) -> bool:
        if self in polygon: 
            return True
        total_vertex = len(polygon)
        cross_check  = [(polygon[(ind + 1) % total_vertex]).cross(self, origin=point) for ind, point in enumerate(polygon)]
        return all(map(lambda cross_product: cross_product <= 0, cross_check))


    def in_circle(self, center: Union[Coordinate, Vector2D], radius: Number) -> bool:
        dx, dy = abs(self - center)

        if dy <= radius and dx <= radius:  return True
        if dy + dx <= radius:              return True
        if dx*dx + dy*dy <= radius*radius: return True

        return False


    def update(self, *vector: Union[Coordinate, Vector2D]):
        try:
            vector = vector if len(vector) > 1 else vector[0]
            self.x, self.y = float(vector[0]), float(vector[1])
        except (ValueError, TypeError, IndexError) as err:
            raise ValueError(f"'{self.__class__.__name__}' accepts 2 arguements, 'X' and 'Y'") from err


    def __add__(self, vector: Union[Coordinate, Vector2D]):
        return Vector2D(self.x + vector[0], self.y + vector[1])


    def __sub__(self, vector: Union[Coordinate, Vector2D]):
        return Vector2D(self.x - vector[0], self.y - vector[1])


    def __mul__(self, val: Number):
        return Vector2D(self.x * val, self.y * val)


    def __truediv__(self, val: Number):
        return Vector2D(self.x / val, self.y / val)


    def __floordiv__(self, val: Number):
        return Vector2D(self.x // val, self.y // val)


    def __round__(self, ndigits: int) -> Vector2D:
        return Vector2D(round(self.x, ndigits), round(self.y, ndigits))


    def __invert__(self) -> Vector2D:
        return Vector2D(self.y, self.x)


    def __abs__(self) -> Vector2D:
        return Vector2D(abs(self.x), abs(self.y))


    def __neg__(self) -> Vector2D:
        return Vector2D(self.x * -1, self.y * -1)

    def __pos__(self):
        return self


    def __ceil__(self) -> Vector2D:
        return Vector2D(ceil(self.x), ceil(self.y))


    def __floor__(self) -> Vector2D:
        return Vector2D(int(floor(self.x)), int(floor(self.y)))


    def __eq__(self, vector: Vector2D) -> bool:
        if isinstance(vector, type(self)) and self.x == vector.x and self.y == vector.y:
            return True
        return False


    def __ne__(self, vector: Vector2D) -> bool:
        if not isinstance(vector, type(self)) or self.x != vector.x or self.y != vector.y:
            return True
        return False


    def __lt__(self, vector: Vector2D) -> bool:
        if not isinstance(vector, type(self)):
            raise TypeError(f"cannot compare '{type(vector).__name__}' with '{self.__class__.__name__}'")
        return True if self.length < vector.length else False


    def __gt__(self, vector: Vector2D) -> bool:
        if not isinstance(vector, type(self)):
            raise TypeError(f"cannot compare '{type(vector).__name__}' with '{self.__class__.__name__}'")
        return True if self.length > vector.length else False


    def __ge__(self, vector: Vector2D) -> bool:
        if not isinstance(vector, type(self)):
            raise TypeError(f"cannot compare '{type(vector).__name__}' with '{self.__class__.__name__}'")
        return True if self.length >= vector.length else False


    def __le__(self, vector: Vector2D) -> bool:
        if not isinstance(vector, type(self)):
            raise TypeError(f"cannot compare '{type(vector).__name__}' with '{self.__class__.__name__}'")
        return True if self.length <= vector.length else False


    def __getitem__(self, index: int) -> Number:
        if index == 0: return self.x
        if index == 1: return self.y

    def __setitem__(self, index: int, val: Number) -> Number:
        if index == 0: self.x = float(val)
        if index == 1: self.y = float(val)


    def __hash__(self):
        return hash(self.astuple)


    def __iter__(self) -> Iterable:
        return iter((self.x, self.y))


    def __str__(self) -> str:
        return f'[{self.x}, {self.y}]'


    def __repr__(self) -> str:
        return f"({self.x}, {self.y})"


    def __copy__(self):
      new_vec = type(self)(self.x, self.y)
      new_vec.__dict__.update(self.__dict__)
      return new_vec


 