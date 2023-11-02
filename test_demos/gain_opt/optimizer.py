import numpy as np
import matplotlib.pyplot as plt


class CEM:
    def __init__(
        self, init_mean, init_cov, eval_func, n_samples=10, n_samples_for_update=5
    ):
        self._mu = np.array(init_mean)
        self._cov = np.array(init_cov)
        self._eval_func = eval_func
        self._n_samples = n_samples
        self._n_samples2update = n_samples_for_update
        self._c_obj_vals = np.zeros(self._n_samples)
        self._idx_c_obj_vals = np.zeros(self._c_obj_vals.shape)
        self.mu_hist = []

    def sample(self):
        return np.random.multivariate_normal(
            mean=self._mu, cov=self._cov, size=self._n_samples
        )

    def _update(self, samples, returns):

        idx_c_obj_vals = np.argsort(returns, axis=0)[::-1]
        update_vales = samples[idx_c_obj_vals[: self._n_samples2update]]
        self._mu = np.mean(update_vales, axis=0)
        self.mu_hist.append(self._mu.copy())
        self._cov = np.atleast_2d(np.cov(update_vales.T))

    def iter(self, n_it):
        for i in range(n_it):
            samples = self.sample()
            returns = self._eval_func(samples)
            self._update(samples, returns)
            print("Iteration: ", i, " Return: ", np.mean(returns), " sol: ", self._mu)
        return self._mu


class QuadFunc:
    @staticmethod
    def eval_func(x):
        return (-(x**2)).squeeze()


if __name__ == "__main__":
    np.random.seed(3)
    eval_func = QuadFunc.eval_func
    cem = CEM(
        init_mean=np.array([3]),
        init_cov=np.eye(1) * 5,
        eval_func=eval_func,
        n_samples=25,
        n_samples_for_update=10,
    )
    res = cem.iter(n_it=20)
    plt.figure()
    x_range = np.linspace(-5, 5, 100)
    y_range = eval_func(x_range)
    plt.plot(x_range, y_range)
    for i, mu in enumerate(cem.mu_hist):
        plt.plot(
            mu, eval_func(mu), "rx", alpha=np.clip((i + 2) / len(cem.mu_hist), 0.1, 1)
        )
    plt.plot(res, eval_func(res), "bx")
