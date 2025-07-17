Extraneous
=================

Extra Thoughts
-----------------
While the current :term:`SLAM` and path planning system is designed to be efficient and robust, several challenges may arise in practice:

    Sensor Noise and Drift: :term:`GPS` and :term:`IMU` measurements can be noisy or suffer from drift. This can lead to inaccurate localization or erroneous map updates.

    :term:`Data Association` Ambiguity: Identifying whether a newly observed cone corresponds to a previously seen cone (:term:`data association`) is challenging in cluttered environments. Misassociations can degrade map quality and cause localization accuracy.

    Tuning Sensitivity: :term:`SLAM` performance heavily depends on parameters such as the :term:`Mahalanobis distance` threshold and noise models. Poorly tuned parameters can result in either missed associations or false positives.

    Failure Recovery: If the system becomes mislocalized or the :term:`factor graph` diverges due to accumulated error, recovering gracefully is non-trivial and may require additional loop closure strategies or reinitialization.

    Real World Edge Cases: Lighting conditions, partial occlusions, or uneven terrain can cause discrepancies between perception and ground truth, which affect downstream :term:`SLAM` and planning modules.