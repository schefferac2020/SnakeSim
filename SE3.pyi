# encoding: utf-8
# module manifpy._bindings
# from /home/quintin/Programming/snake-sim/venv/lib/python3.10/site-packages/manifpy/_bindings.cpython-310-x86_64-linux-gnu.so
# by generator 1.147
""" Python bindings for the manif library, a small library for Lie theory. """

# imports
import pybind11_builtins as __pybind11_builtins


from ._SE3Base import _SE3Base

class SE3(_SE3Base):
    # no doc
    def act(self, p, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        act(self: manifpy._bindings.SE3, p: numpy.ndarray[numpy.float64[3, 1]], J_out_self: Optional[numpy.ndarray[numpy.float64[3, 6], flags.writeable, flags.c_contiguous]] = None, J_out_p: Optional[numpy.ndarray[numpy.float64[3, 3], flags.writeable, flags.c_contiguous]] = None) -> numpy.ndarray[numpy.float64[3, 1]]
        
        
              Get the action of the Lie group object on a point.
        
              Parameters
              ----------
              p : numpy.array
                A point.
              J_out_self [out] : numpy.ndarray
                Jacobian of the new object wrt self.
              J_out_p [out] : numpy.ndarray
                Jacobian of the new object wrt input point.
        """
        pass

    def adj(self): # real signature unknown; restored from __doc__
        """
        adj(self: manifpy._bindings.SE3) -> numpy.ndarray[numpy.float64[6, 6]]
        
        
              Return the Adjoint of the Lie group object self.
        
              See Eq. (29).
        """
        pass

    def between(self, other, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        between(self: manifpy._bindings.SE3, other: manifpy._bindings._SE3Base, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_out_other: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3
        
        
              Return the between of self and another object of the same Lie group.
        
              Parameters
              ----------
              other : Lie group
                Another object of the same Lie group.
              J_out_self [out] : numpy.ndarray
                Jacobian of the composition wrt self.
              J_out_other [out] : numpy.ndarray
                Jacobian of the composition wrt other.
        """
        pass

    def coeffs(self): # real signature unknown; restored from __doc__
        """
        coeffs(self: manifpy._bindings.SE3) -> numpy.ndarray[numpy.float64[7, 1]]
        
        Get a reference to underlying data.
        """
        pass

    def coeffs_copy(self): # real signature unknown; restored from __doc__
        """
        coeffs_copy(self: manifpy._bindings.SE3) -> numpy.ndarray[numpy.float64[7, 1]]
        
        Return a copy of underlying data.
        """
        pass

    def compose(self, other, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        compose(self: manifpy._bindings.SE3, other: manifpy._bindings._SE3Base, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_out_other: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3
        
        
              Return the composition of self and another object of the same Lie group.
        
              See Eqs. (1,2,3,4).
        
              Parameters
              ----------
              other : Lie group
                Another object of the same Lie group.
              J_out_self [out] : numpy.ndarray
                Jacobian of the composition wrt self.
              J_out_other [out] : numpy.ndarray
                Jacobian of the composition wrt other.
        """
        pass

    def Identity(self): # real signature unknown; restored from __doc__
        """
        Identity() -> manifpy._bindings.SE3
        
        Static helper to create an object set at the Lie group Identity.
        """
        pass

    def inverse(self, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        inverse(self: manifpy._bindings.SE3, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3
        
        
              Return the inverse of the Lie group object.
        
              See Eq. (3).
        
              Parameters
              ----------
              J_out_self [out] : numpy.ndarray
                Jacobian of the inverse wrt self.
        """
        pass

    def isApprox(self, other, eps=1, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        isApprox(self: manifpy._bindings.SE3, other: manifpy._bindings._SE3Base, eps: float = 1e-10) -> bool
        
        
              Evaluate whether self and other are 'close'.
        
              Parameters
              ----------
              other : Lie group
                Another object of the same Lie group.
              eps : double
                Threshold for equality comparison. Default: 1e-10.
        """
        pass

    def lminus(self, other, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        lminus(self: manifpy._bindings.SE3, other: manifpy._bindings._SE3Base, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_out_other: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3Tangent
        
        
              Left ominus operation of the Lie group.
        
              See Eq. (28).
        
              Parameters
              ----------
              other : Lie group
                Another element of the same Lie group.
              J_out_self [out] : numpy.ndarray
                Jacobian of the ominus operation wrt self.
              J_out_other [out] : numpy.ndarray
                Jacobian of the ominus operation wrt other.
        """
        pass

    def log(self, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        log(self: manifpy._bindings.SE3, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3Tangent
        
        
              Return the corresponding Lie algebra element in vector form.
        
              Eq. (24).
        
              Parameters
              ----------
              J_out_self [out] : numpy.ndarray
                Jacobian of the log wrt self.
        """
        pass

    def lplus(self, tau, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        lplus(self: manifpy._bindings.SE3, tau: manifpy._bindings._SE3TangentBase, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_mout_tau: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3
        
        
              Left oplus operation of the Lie group.
        
              See Eq. (27).
        
              Parameters
              ----------
              tau : Lie group tangent
                An element of the tangent of the Lie group.
              J_out_self [out] : numpy.ndarray
                Jacobian of the oplus operation wrt self.
              J_out_tau [out] : numpy.ndarray
                Jacobian of the oplus operation wrt tau.
        """
        pass

    def minus(self, other, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        minus(self: manifpy._bindings.SE3, other: manifpy._bindings._SE3Base, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_out_other: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3Tangent
        
        An alias for the 'rminus' function.
        """
        pass

    def normalize(self): # real signature unknown; restored from __doc__
        """ normalize(self: manifpy._bindings.SE3) -> None """
        pass

    def plus(self, tau, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        plus(self: manifpy._bindings.SE3, tau: manifpy._bindings._SE3TangentBase, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_mout_tau: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3
        
        An alias for the 'rplus' function.
        """
        pass

    def quat(self, *args, **kwargs): # real signature unknown; restored from __doc__
        """
        quat(*args, **kwargs)
        Overloaded function.
        
        1. quat(self: manifpy._bindings.SE3) -> numpy.ndarray[numpy.float64[4, 1]]
        
        2. quat(self: manifpy._bindings.SE3, quaternion: numpy.ndarray[numpy.float64[4, 1]]) -> None
        """
        pass

    def Random(self): # real signature unknown; restored from __doc__
        """
        Random() -> manifpy._bindings.SE3
        
        Static helper to create a random object of the Lie group.
        """
        pass

    def rminus(self, other, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        rminus(self: manifpy._bindings.SE3, other: manifpy._bindings._SE3Base, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_out_other: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3Tangent
        
        
              Right ominus operation of the Lie group.
        
              See Eq. (26).
        
              Parameters
              ----------
              other : Lie group
                Another element of the same Lie group.
              J_out_self [out] : numpy.ndarray
                Jacobian of the ominus operation wrt self.
              J_out_other [out] : numpy.ndarray
                Jacobian of the ominus operation wrt other.
        """
        pass

    def rotation(self): # real signature unknown; restored from __doc__
        """ rotation(self: manifpy._bindings.SE3) -> numpy.ndarray[numpy.float64[3, 3]] """
        pass

    def rplus(self, tau, J_out_self, *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
        """
        rplus(self: manifpy._bindings.SE3, tau: manifpy._bindings._SE3TangentBase, J_out_self: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None, J_out_tau: Optional[numpy.ndarray[numpy.float64[6, 6], flags.writeable, flags.c_contiguous]] = None) -> manifpy._bindings.SE3
        
        
              Right oplus operation of the Lie group.
        
              See Eq. (25).
        
              Parameters
              ----------
              tau : Lie group tangent
                An element of the tangent of the Lie group.
              J_out_self [out] : numpy.ndarray
                Jacobian of the oplus operation wrt self.
              J_out_tau [out] : numpy.ndarray
                Jacobian of the oplus operation wrt tau.
        """
        pass

    def setIdentity(self): # real signature unknown; restored from __doc__
        """
        setIdentity(self: manifpy._bindings.SE3) -> manifpy._bindings.SE3
        
        Set self to the Lie group Identity.
        """
        pass

    def setRandom(self): # real signature unknown; restored from __doc__
        """
        setRandom(self: manifpy._bindings.SE3) -> manifpy._bindings.SE3
        
        Set self to a random value.
        """
        pass

    def transform(self): # real signature unknown; restored from __doc__
        """ transform(self: manifpy._bindings.SE3) -> numpy.ndarray[numpy.float64[4, 4]] """
        pass

    def translation(self, *args, **kwargs): # real signature unknown; restored from __doc__
        """
        translation(*args, **kwargs)
        Overloaded function.
        
        1. translation(self: manifpy._bindings.SE3) -> numpy.ndarray[numpy.float64[3, 1]]
        
        2. translation(self: manifpy._bindings.SE3, translation: numpy.ndarray[numpy.float64[3, 1]]) -> None
        """
        pass

    def x(self): # real signature unknown; restored from __doc__
        """ x(self: manifpy._bindings.SE3) -> float """
        return 0.0

    def y(self): # real signature unknown; restored from __doc__
        """ y(self: manifpy._bindings.SE3) -> float """
        return 0.0

    def z(self): # real signature unknown; restored from __doc__
        """ z(self: manifpy._bindings.SE3) -> float """
        return 0.0

    def __add__(self, arg0): # real signature unknown; restored from __doc__
        """
        __add__(self: manifpy._bindings.SE3, arg0: manifpy._bindings.SE3Tangent) -> manifpy._bindings.SE3
        
        Operator overload for the 'plus' function.
        """
        pass

    def __eq__(self, arg0): # real signature unknown; restored from __doc__
        """
        __eq__(self: manifpy._bindings.SE3, arg0: manifpy._bindings.SE3) -> bool
        
        Operator overload for the 'isApprox' function.
        """
        return False

    def __init__(self, *args, **kwargs): # real signature unknown; restored from __doc__
        """
        __init__(*args, **kwargs)
        Overloaded function.
        
        1. __init__(self: manifpy._bindings.SE3) -> None
        
        Default constructor, uninitialized data.
        
        2. __init__(self: manifpy._bindings.SE3, arg0: numpy.ndarray[numpy.float64[7, 1]]) -> None
        
        Constructor given data vector.
        
        3. __init__(self: manifpy._bindings.SE3, arg0: float, arg1: float, arg2: float, arg3: float, arg4: float, arg5: float) -> None
        
        4. __init__(self: manifpy._bindings.SE3, position: numpy.ndarray[numpy.float64[3, 1]], quaternion: numpy.ndarray[numpy.float64[4, 1]]) -> None
        """
        pass

    def __mul__(self, arg0): # real signature unknown; restored from __doc__
        """
        __mul__(self: manifpy._bindings.SE3, arg0: manifpy._bindings.SE3) -> manifpy._bindings.SE3
        
        Operator overload for the 'compose' function.
        """
        pass

    def __str__(self): # real signature unknown; restored from __doc__
        """ __str__(self: manifpy._bindings.SE3) -> str """
        return ""

    def __sub__(self, arg0): # real signature unknown; restored from __doc__
        """
        __sub__(self: manifpy._bindings.SE3, arg0: manifpy._bindings.SE3) -> manifpy._bindings.SE3Tangent
        
        Operator overload for the 'minus' function.
        """
        pass

    Dim = 3
    DoF = 6
    RepSize = 7
    __hash__ = None


