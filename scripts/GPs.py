from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import matplotlib.colors as colors
import numpy as np
import scipy as sc
import scipy.integrate
import warnings
import tqdm as tqdm

def plot_ellipse(ax, mean, cov, n_std=2, color="tab:red", alpha=.2, border=False, **kwargs):
    if cov[0,0] < 1e-5**2 or cov[1,1] < 1e-5**2:
        return
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensionl dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0),
        width=ell_radius_x * 2,
        height=ell_radius_y * 2,
        facecolor= (colors.to_rgba(color, alpha) if border==False else (0,0,0,0)),
        edgecolor= (colors.to_rgba(color, alpha) if border==True else (0,0,0,0)),
        **kwargs)

    # Calculating the stdandard deviation of x from
    # the squareroot of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    mean_x = mean[0]

    # calculating the stdandard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std
    mean_y = mean[1]

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)

    # ax.scatter(*mean, marker='x', color=color)
    return ax.add_patch(ellipse)

class LTI_system:
    def __init__(self, A, B, L, Q, lQk=None, int_exp=None):
        self.A = A
        self.B = B
        self.L = L
        self.Q = Q
        self.Phi = lambda t,s: sc.linalg.expm(A*(t-s))

        if int_exp is None and not np.isfinite(np.linalg.cond(self.A)):
            #mandatory when A is not invertible for real time applications.
            #if A is invertible, it is not used.
            warnings.warn("Warning: A is not invertible, performing numerical integration, this will take longer. \
                          Please give a lambda |R->|R in the constructor that solves in constant time the integral int_0^dt exp( (t-s)A)ds. \
                          Such a function always exists and should be pretty straightforward to find.")
            self.int_exp = lambda dt: sc.integrate.quad_vec(lambda s: sc.linalg.expm(self.A * (dt-s)), 0, dt)[0]
        else:
            self.int_exp = int_exp

        if lQk is None:
            warnings.warn("Warning: lQk is not given, performing numerical integration, this will take longer. \
                            Please give a closed form of equation (3,212) for better performances")
            self.lQk = lambda dt:  sc.integrate.quad_vec(
                                    lambda s: sc.linalg.expm(self.A * (dt-s)) @ self.L @ self.Q @ self.L.T @ sc.linalg.expm(self.A * (dt-s)).T,
                                    0, dt)[0]

        self.lA = None
        self.lB = None

    def compute_lA(self, ts):
        #compute the lifted matrix A
        lA = np.eye(ts.shape[0]*self.A.shape[0], ts.shape[0]*self.A.shape[1])
        for i in range(1,ts.shape[0]):
            for j in range(0,i):
                i0 = i*self.A.shape[0]
                i1 = (i+1)*self.A.shape[0]
                j0 = j*self.A.shape[1]
                j1 = (j+1)*self.A.shape[1]
                lA[i0:i1,j0:j1] = self.Phi(ts[i], ts[j])

        self.lA = lA #save for optimization
        return lA

    def compute_lB(self, ts):
        #compute the lifted matrix B
        lB = np.eye(self.A.shape[0] + self.A.shape[0]*(ts.shape[0]-1),
                    self.A.shape[0] + self.B.shape[1]*(ts.shape[0]-1))

        if np.isfinite(np.linalg.cond(self.A)):
            iA = np.linalg.inv(self.A)

        Bks = []
        for i in range(0,ts.shape[0]-1):
            i0 = self.A.shape[0] + i*self.A.shape[0]
            i1 = self.A.shape[0] + (i+1)*self.A.shape[0]
            j0 = self.A.shape[0] + i*self.B.shape[1]
            j1 = self.A.shape[0] + (i+1)*self.B.shape[1]

            if np.isfinite(np.linalg.cond(self.A)):
                p = self.Phi(ts[i+1], ts[i])
                Bks +=  [p @ (np.eye(*p.shape) - np.linalg.inv(p)) @ iA @ self.B]
            else:
                #not invertible, the closed-form is not good, need more information from user
                #cf https://math.stackexchange.com/questions/658276/integral-of-matrix-exponential
                dt = ts[i+1]-ts[i]
                Bks += [self.int_exp(dt) @ self.B]

        self.lB = sc.linalg.block_diag(np.eye(self.A.shape[0], self.A.shape[0]), *Bks) #save for optimization
        return self.lB


    def compute_lQ(self, ts, P0):
        lQ = np.zeros((P0.shape[0] + self.A.shape[0]*(ts.shape[0]-1), #P0.shape == A.shape
                      P0.shape[0] + self.A.shape[1]*(ts.shape[0]-1)))
        for i in range(0, ts.shape[0]):
            i0 = i*self.A.shape[0]
            i1 = (i+1)*self.A.shape[0]
            j0 = i*self.A.shape[1]
            j1 = (i+1)*self.A.shape[1]

            if i==0:
                lQ[i0:i1, j0:j1] = P0
            else: #i>0
                lQ[i0:i1, j0:j1] = self.lQk(ts[i]-ts[i-1])
        self.lQ = lQ
        return lQ

    def compute_mean(self, ts, u, x0=None, recompute=True):
        #u is assumed to be constant between each pair of consecutive measurements times
        if u.shape[0] != (ts.shape[0]-1)*self.B.shape[1]:
             raise ValueError('for T times, u should contain T-1 commands. u[i] is the applied (constant) command between times ts[i] and ts[i+1]')
        if x0 is None:
            x0 = np.zeros(self.A.shape[1])

        lu = np.concatenate([x0, u]) #[x0, u1, u2, ..., uM]

        if recompute or self.lA is None or self.lB is None:
            self.compute_lA(ts)
            self.compute_lB(ts)

        self.mean = self.lA @ self.lB @ lu #useful but a bit slow, should optimize sparse matrices multiplications
        return self.mean

    def compute_covariance(self, ts, u, P0=None, recompute=True):
        if P0 is None:
            P0 = np.eye(*self.A.shape[0:2])*1e-5**2

        if recompute or self.lQ is None:
            self.compute_lA(ts)
            self.compute_lQ(ts, P0)

        self.covariance = self.lA @ self.lQ @ self.lA.T

        return self.covariance

    def query_mean(self, ts, u, tau):
        if self.mean is None or self.covariance is None:
            raise ValueError('Please compute mean and covariance before querying')

        #u is assumed to be constant between each pair of consecutive measurements times
        if u.shape[0] != (ts.shape[0]-1)*self.B.shape[1]:
             raise ValueError('for T times, u should contain T-1 commands. u[i] is the applied (constant) command between times ts[i] and ts[i+1]')

        i = 0
        while i<len(ts) and ts[i]<=tau:
            i+=1
        i-=1
        if i<0: #TODO implement this case using x0
            raise ValueError('Please query for a time after the first measurement')
        if i==len(ts)-1:
            i-=1
            warnings.warn("warning: extrapolate after last command, assuming constant command")

        p = self.Phi(tau, ts[i])
        if np.isfinite(np.linalg.cond(self.A)):
                lB =  p @ (np.eye(*p.shape) - np.linalg.inv(p)) @ np.linalg.inv(self.A) @ self.B
        else:
            #not invertible, the closed-form is not good, need more information from user
            #cf https://math.stackexchange.com/questions/658276/integral-of-matrix-exponential
            dt = tau-ts[i]
            lB = self.int_exp(dt) @ self.B

        return p @ self.mean[self.A.shape[1]*i:self.A.shape[1]*(i+1)] + lB@u[self.B.shape[1]*i:self.B.shape[1]*(i+1)]

    def query_covariance(self, ts, tau):
        if self.mean is None or self.covariance is None:
            raise ValueError('Please compute mean and covariance before querying')

        i = 0
        while i<len(ts) and ts[i]<=tau:
            i+=1
        i-=1
        if i<0: #TODO implement this case using x0
            raise ValueError('Please query for a time after the first measurement')
        if i==len(ts)-1:
            i-=1
            warnings.warn("warning: extrapolate after last command, assuming constant command")

        p = self.Phi(tau, ts[i])
        i0 = i*self.A.shape[0]
        i1 = (i+1)*self.A.shape[0]

        return p @ self.covariance[i0:i1, i0:i1] @ p.T + self.lQk(tau-ts[i])

class GPs_LTI:
    def __init__(self, lti_system, ts, us, measurements, C, R, P0=None, x0=None):
        self.C = C
        self.R = R
        self.sys = lti_system
        self.x0 = x0
        self.P0 = P0

        self.prior_mean = None
        self.prior_cov = None
        self.prior_cov_i = None

        self.post_mean = None
        self.post_cov = None

        self.ts = ts
        self.us = us
        self.measurements = measurements

    def compute_prior(self):
        self.prior_mean = self.sys.compute_mean(self.ts, self.us, x0=self.x0)
        self.prior_cov  = self.sys.compute_covariance(self.ts, self.us, P0=self.P0)
        self.prior_cov_i = np.linalg.inv(self.prior_cov)

    def infer_mean(self, recompute=True):
        #TODO part of the expression for the mean is simply the posterior covariance, we could avoir computing it twice
        if recompute or self.prior_mean is None or  self.prior_cov is None or self.prior_cov_i is None:
            self.compute_prior()

        Ri = np.linalg.inv(self.R)

        self.post_mean = ( np.linalg.inv(self.prior_cov_i + self.C.T@Ri@self.C) @
                          (self.prior_cov_i@self.prior_mean + self.C.T@Ri@self.measurements))

        return self.post_mean

    def infer_covariance(self, recompute=True):
        if recompute or self.prior_mean is None or  self.prior_cov is None or self.prior_cov_i is None:
            self.compute_prior()

        Ri = np.linalg.inv(self.R)

        self.post_cov = np.linalg.inv(self.prior_cov_i + self.C.T@Ri@self.C)

        return self.post_cov

    def query_posterior(self, tau):
        if self.post_mean is None or self.post_cov is None:
            raise ValueError('Please first infer mean and covariance before querying the GP.')

        i = 0
        while i<len(self.ts) and self.ts[i]<=tau:
            i+=1
        i-=1
        if i<0: #TODO implement this case using x0
            raise ValueError('Please query for a time after the first measurement')
        if i==len(self.ts)-1:
            raise ValueError('Please query for a time before the last measurement')

        tk = self.ts[i]
        tk1 = self.ts[i+1]

        Qtau = self.sys.lQk(tau-tk)
        Qk1i = np.linalg.inv(self.sys.lQk(tk1-tk))

        Lambda = self.sys.Phi(tau, tk) - Qtau @ self.sys.Phi(tk1,tau).T @ Qk1i @ self.sys.Phi(tk1, tk)
        varPhi = Qtau @ self.sys.Phi(tk1, tau).T @ Qk1i

        LP = np.block([Lambda, varPhi])
        LPT = np.block([[Lambda.T],
                        [varPhi.T]])

        #Posterior Mean

        x_prior = self.sys.query_mean(self.ts, self.us, tau)

        s = self.sys.A.shape[0]
        x_hat = np.concatenate([self.post_mean[i*s:(i+1)*s], self.post_mean[(i+1)*s:(i+2)*s]])
        x_vee = np.concatenate([self.sys.mean[i*s:(i+1)*s], self.sys.mean[(i+1)*s:(i+2)*s]])

        x_post = x_prior + LP @ (x_hat - x_vee)

        #Posterior Cov
        P_prior= self.sys.query_covariance(self.ts, tau)
        P_vee_kk   = self.sys.covariance[i*s:(i+1)*s, i*s:(i+1)*s]
        P_vee_kk1  = self.sys.covariance[i*s:(i+1)*s, (i+1)*s:(i+2)*s]
        P_vee_k1k  = self.sys.covariance[(i+1)*s:(i+2)*s, i*s:(i+1)*s]
        P_vee_k1k1 = self.sys.covariance[(i+1)*s:(i+2)*s, (i+1)*s:(i+2)*s]
        P_hat_kk   = self.post_cov[i*s:(i+1)*s, i*s:(i+1)*s]
        P_hat_kk1  = self.post_cov[i*s:(i+1)*s, (i+1)*s:(i+2)*s]
        P_hat_k1k  = self.post_cov[(i+1)*s:(i+2)*s, i*s:(i+1)*s]
        P_hat_k1k1 = self.post_cov[(i+1)*s:(i+2)*s, (i+1)*s:(i+2)*s]

        P_hat = np.block([[P_hat_kk,  P_hat_kk1],
                          [P_hat_k1k, P_hat_k1k1]])
        P_vee = np.block([[P_vee_kk,  P_vee_kk1],
                          [P_vee_k1k, P_vee_k1k1]])

        P_post = P_prior + LP @ (P_hat - P_vee) @ LPT

        return x_post, P_post

class GPs_LTI_nL_m(GPs_LTI):
    def __init__(self, lti_system, ts, us, measurements, g, G, R, P0=None, x0=None, initial_guess=None, max_it=30, rtol=1e-5):
        #G is now a lambda that returns the gradient of the measuremnt function for a given trajectory at each time
        super().__init__(lti_system, ts, us, measurements, None, R, P0, x0)

        self.g = g
        self.G = G

        self.initial_guess = np.array(initial_guess)
        self.max_it = max_it
        self.rtol=rtol

    def infer_mean(self, recompute=True):
        #cf 4.4.4 of Barfoot book
        if recompute or self.prior_mean is None or  self.prior_cov is None or self.prior_cov_i is None:
            self.compute_prior()

        if self.initial_guess is None:
            self.initial_guess = np.array(self.prior_mean)

        Ri = np.linalg.inv(self.R)
        lB = self.sys.compute_lB(self.ts)
        lu = np.concatenate([self.x0, self.us]) #[x0, u1, u2, ..., uM]

        #1) start with initial guess
        x_op = np.array(self.initial_guess)

        #2) skip this step as motion equation is linear

        pbar = tqdm.tqdm(range(self.max_it))
        for i in pbar:
            #3) compute G and y_op
            G = self.G(x_op)
            y_op = self.g(x_op)

            #4 solve for dx the equation (4.206)
            lhs = self.prior_cov_i + self.G(x_op).T @ Ri @ self.G(x_op)

            rhs = self.prior_cov_i @ (lB @ lu - x_op) + self.G(x_op).T @ Ri @ (self.measurements - y_op)

            dx = np.linalg.inv(lhs) @ rhs

            x_op += dx

            gain = np.linalg.norm(dx)/np.linalg.norm(x_op)
            pbar.set_description("Iteration [{}]: {:.4f} % gain |=========================| ".format(i, gain*100))

            if gain < self.rtol:
                print("Optimization finished!")
                break

        if gain > self.rtol:
            warnings.warn("warning: The optimizer did not converge!")

        self.C = self.G(x_op)

        self.post_mean = x_op

        return x_op
