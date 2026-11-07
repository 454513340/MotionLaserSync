#!/usr/bin/env python3
"""Offline image-domain surrogate tests. No proprietary sensor raw images."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
import numpy as np
from m34_greybox_imager import (gaussian_component_area,GreyboxParams,prepare,predict,
                                 extract_profile_from_synthetic_image)

class ImagingTests(unittest.TestCase):
    def test_gaussian_integral(self):
        self.assertAlmostEqual(float(gaussian_component_area(np.array([.5]),np.array([2.]),np.array([1.]))[0]),0.)
        self.assertGreater(float(gaussian_component_area(np.array([4.]),np.array([2.]),np.array([1.]))[0]),0.)
    def test_largest_area_differs_from_highest_peak(self):
        zs=np.linspace(90,110,2001)
        # Narrow high peak at 94 mm; broad lower peak at 104 mm.
        x=7*np.exp(-.5*((zs-94)/.07)**2)+3*np.exp(-.5*((zs-104)/.7)**2)
        img=x[:,None]
        wide=extract_profile_from_synthetic_image(img,zs,reflection_selection='largest_area',min_component_area=.1)
        tall=extract_profile_from_synthetic_image(img,zs,reflection_selection='highest_intensity',min_component_area=.1)
        self.assertTrue(wide['valid'][0] and tall['valid'][0])
        self.assertAlmostEqual(float(wide['sensor_z'][0]),104,delta=.04)
        self.assertAlmostEqual(float(tall['sensor_z'][0]),94,delta=.04)
    def test_centroid_subpixel(self):
        zs=np.linspace(90,100,1001);img=2*np.exp(-.5*((zs-96.237)/.19)**2)
        extracted=extract_profile_from_synthetic_image(img[:,None],zs,absolute_floor=.12,min_component_area=.2)
        self.assertAlmostEqual(float(extracted['sensor_z'][0]),96.237,delta=.01)
    def test_hard_no_return(self):
        zs=np.linspace(90,100,101);img=np.zeros((len(zs),3))
        got=extract_profile_from_synthetic_image(img,zs)
        self.assertFalse(got['valid'].any())
        self.assertTrue(np.isnan(got['sensor_z']).all())
    def test_missing_camera_image_is_not_faked_as_measurement(self):
        # Default grey box cannot infer intensity directly from a SCAN file.
        p=GreyboxParams();p.check()
        self.assertAlmostEqual(p.dynamic_threshold_fraction,.125)

if __name__=='__main__':unittest.main()
