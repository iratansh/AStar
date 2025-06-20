#!/usr/bin/env python3
"""
Sensor Processing Scripts for Behavioral Study
Processes EmotiBit (HRV/EDA), Eye-tracking, and fNIRS data
Synchronized with robot behavioral events
"""

import numpy as np
import pandas as pd
import scipy.signal
from scipy import stats
from typing import Dict, List, Tuple, Optional, Any
import json
import time
from datetime import datetime, timedelta
import warnings
warnings.filterwarnings('ignore')

# NeuroKit2 processing (install with: pip install neurokit2)
try:
    import neurokit2 as nk
    NEUROKIT_AVAILABLE = True
except ImportError:
    NEUROKIT_AVAILABLE = False
    print("NeuroKit2 not available. Install with: pip install neurokit2")

class HRVProcessor:
    """
    Heart Rate Variability processing using NeuroKit2
    Computes RMSSD, pNN50, LF/HF ratio and other HRV indicators
    """
    
    def __init__(self, sampling_rate: int = 1000):
        self.sampling_rate = sampling_rate
        self.processed_data = {}
        
    def process_ecg_signal(self, ecg_data: np.ndarray, timestamp_data: np.ndarray = None) -> Dict:
        """
        Process ECG signal to extract HRV metrics
        
        Args:
            ecg_data: ECG signal array
            timestamp_data: Corresponding timestamps
            
        Returns:
            Dictionary containing HRV metrics
        """
        if not NEUROKIT_AVAILABLE:
            return self._fallback_hrv_processing(ecg_data)
        
        try:
            # Clean ECG signal
            ecg_cleaned = nk.ecg_clean(ecg_data, sampling_rate=self.sampling_rate)
            
            # Find R-peaks
            r_peaks, _ = nk.ecg_peaks(ecg_cleaned, sampling_rate=self.sampling_rate)
            
            # Calculate HRV metrics
            hrv_time = nk.hrv_time(r_peaks, sampling_rate=self.sampling_rate)
            hrv_freq = nk.hrv_frequency(r_peaks, sampling_rate=self.sampling_rate)
            
            # Extract key metrics
            results = {
                'timestamp': time.time() if timestamp_data is None else timestamp_data[-1],
                'heart_rate_mean': np.mean(nk.ecg_rate(r_peaks, sampling_rate=self.sampling_rate)),
                'rmssd': hrv_time['HRV_RMSSD'].iloc[0] if 'HRV_RMSSD' in hrv_time.columns else None,
                'pnn50': hrv_time['HRV_pNN50'].iloc[0] if 'HRV_pNN50' in hrv_time.columns else None,
                'sdnn': hrv_time['HRV_SDNN'].iloc[0] if 'HRV_SDNN' in hrv_time.columns else None,
                'lf_power': hrv_freq['HRV_LF'].iloc[0] if 'HRV_LF' in hrv_freq.columns else None,
                'hf_power': hrv_freq['HRV_HF'].iloc[0] if 'HRV_HF' in hrv_freq.columns else None,
                'lf_hf_ratio': hrv_freq['HRV_LFHF'].iloc[0] if 'HRV_LFHF' in hrv_freq.columns else None,
                'total_power': hrv_freq['HRV_TP'].iloc[0] if 'HRV_TP' in hrv_freq.columns else None,
                'r_peaks_count': len(r_peaks),
                'signal_quality': self._assess_signal_quality(ecg_cleaned, r_peaks)
            }
            
            return results
            
        except Exception as e:
            print(f"Error processing ECG: {e}")
            return self._fallback_hrv_processing(ecg_data)
    
    def _fallback_hrv_processing(self, ecg_data: np.ndarray) -> Dict:
        """Fallback HRV processing when NeuroKit2 is not available"""
        # Simple peak detection
        peaks, _ = scipy.signal.find_peaks(ecg_data, height=np.std(ecg_data))
        
        if len(peaks) < 2:
            return {'error': 'Insufficient peaks detected'}
        
        # Calculate RR intervals
        rr_intervals = np.diff(peaks) / self.sampling_rate * 1000  # Convert to ms
        
        # Basic HRV metrics
        results = {
            'timestamp': time.time(),
            'heart_rate_mean': 60000 / np.mean(rr_intervals) if len(rr_intervals) > 0 else None,
            'rmssd': np.sqrt(np.mean(np.diff(rr_intervals)**2)) if len(rr_intervals) > 1 else None,
            'sdnn': np.std(rr_intervals) if len(rr_intervals) > 0 else None,
            'pnn50': np.sum(np.abs(np.diff(rr_intervals)) > 50) / len(rr_intervals) * 100 if len(rr_intervals) > 1 else None,
            'r_peaks_count': len(peaks),
            'signal_quality': 'estimated'
        }
        
        return results
    
    def _assess_signal_quality(self, ecg_signal: np.ndarray, r_peaks: np.ndarray) -> str:
        """Assess ECG signal quality"""
        if len(r_peaks) == 0:
            return 'poor'
        
        # Check signal-to-noise ratio
        signal_power = np.var(ecg_signal)
        noise_estimate = np.var(np.diff(ecg_signal))
        snr = signal_power / noise_estimate if noise_estimate > 0 else 0
        
        if snr > 10:
            return 'excellent'
        elif snr > 5:
            return 'good'
        elif snr > 2:
            return 'fair'
        else:
            return 'poor'

class EDAProcessor:
    """
    Electrodermal Activity (EDA) processing
    Detects SCR peaks and tonic EDA shifts (stress indicators)
    """
    
    def __init__(self, sampling_rate: int = 1000):
        self.sampling_rate = sampling_rate
        
    def process_eda_signal(self, eda_data: np.ndarray, timestamp_data: np.ndarray = None) -> Dict:
        """
        Process EDA signal to extract stress indicators
        
        Args:
            eda_data: EDA signal array
            timestamp_data: Corresponding timestamps
            
        Returns:
            Dictionary containing EDA metrics
        """
        if not NEUROKIT_AVAILABLE:
            return self._fallback_eda_processing(eda_data)
        
        try:
            # Clean EDA signal
            eda_cleaned = nk.eda_clean(eda_data, sampling_rate=self.sampling_rate)
            
            # Decompose into phasic and tonic components
            eda_decomposed = nk.eda_phasic(eda_cleaned, sampling_rate=self.sampling_rate)
            
            # Find SCR peaks
            scr_peaks, _ = nk.eda_peaks(eda_decomposed['EDA_Phasic'], 
                                      sampling_rate=self.sampling_rate)
            
            # Calculate metrics
            results = {
                'timestamp': time.time() if timestamp_data is None else timestamp_data[-1],
                'eda_mean': np.mean(eda_cleaned),
                'eda_std': np.std(eda_cleaned),
                'tonic_mean': np.mean(eda_decomposed['EDA_Tonic']),
                'tonic_slope': self._calculate_slope(eda_decomposed['EDA_Tonic']),
                'phasic_mean': np.mean(eda_decomposed['EDA_Phasic']),
                'phasic_peaks': len(scr_peaks),
                'scr_frequency': len(scr_peaks) / (len(eda_data) / self.sampling_rate),
                'max_scr_amplitude': np.max(eda_decomposed['EDA_Phasic']) if len(eda_decomposed['EDA_Phasic']) > 0 else 0,
                'signal_quality': self._assess_eda_quality(eda_cleaned)
            }
            
            return results
            
        except Exception as e:
            print(f"Error processing EDA: {e}")
            return self._fallback_eda_processing(eda_data)
    
    def _fallback_eda_processing(self, eda_data: np.ndarray) -> Dict:
        """Fallback EDA processing when NeuroKit2 is not available"""
        # Simple smoothing
        eda_smooth = scipy.signal.savgol_filter(eda_data, window_length=51, polyorder=3)
        
        # Simple peak detection for SCR
        threshold = np.mean(eda_smooth) + 2 * np.std(eda_smooth)
        peaks, _ = scipy.signal.find_peaks(eda_smooth, height=threshold)
        
        results = {
            'timestamp': time.time(),
            'eda_mean': np.mean(eda_data),
            'eda_std': np.std(eda_data),
            'tonic_mean': np.mean(eda_smooth),
            'tonic_slope': self._calculate_slope(eda_smooth),
            'scr_frequency': len(peaks) / (len(eda_data) / self.sampling_rate),
            'signal_quality': 'estimated'
        }
        
        return results
    
    def _calculate_slope(self, signal: np.ndarray) -> float:
        """Calculate the slope of a signal using linear regression"""
        if len(signal) < 2:
            return 0.0
        
        x = np.arange(len(signal))
        slope, _, _, _, _ = stats.linregress(x, signal)
        return slope
    
    def _assess_eda_quality(self, eda_signal: np.ndarray) -> str:
        """Assess EDA signal quality"""
        # Check for saturation
        if np.any(eda_signal >= 0.99 * np.max(eda_signal)):
            return 'saturated'
        
        # Check for noise level
        noise_level = np.std(np.diff(eda_signal))
        signal_level = np.std(eda_signal)
        
        if noise_level / signal_level < 0.1:
            return 'excellent'
        elif noise_level / signal_level < 0.2:
            return 'good'
        else:
            return 'noisy'

class EyeTrackingProcessor:
    """
    Eye-tracking data processing
    Analyzes pupil diameter, fixation patterns, and saccade metrics
    """
    
    def __init__(self, sampling_rate: int = 1000):
        self.sampling_rate = sampling_rate
        
    def process_eye_data(self, eye_data: Dict, timestamp_data: np.ndarray = None) -> Dict:
        """
        Process eye-tracking data
        
        Args:
            eye_data: Dictionary containing gaze_x, gaze_y, pupil_diameter arrays
            timestamp_data: Corresponding timestamps
            
        Returns:
            Dictionary containing eye-tracking metrics
        """
        try:
            gaze_x = np.array(eye_data.get('gaze_x', []))
            gaze_y = np.array(eye_data.get('gaze_y', []))
            pupil_diameter = np.array(eye_data.get('pupil_diameter', []))
            
            # Pupil metrics
            pupil_metrics = self._analyze_pupil_data(pupil_diameter)
            
            # Fixation analysis
            fixation_metrics = self._analyze_fixations(gaze_x, gaze_y)
            
            # Saccade analysis
            saccade_metrics = self._analyze_saccades(gaze_x, gaze_y)
            
            # Combine all metrics
            results = {
                'timestamp': time.time() if timestamp_data is None else timestamp_data[-1],
                **pupil_metrics,
                **fixation_metrics,
                **saccade_metrics
            }
            
            return results
            
        except Exception as e:
            print(f"Error processing eye-tracking data: {e}")
            return {'error': str(e)}
    
    def _analyze_pupil_data(self, pupil_diameter: np.ndarray) -> Dict:
        """Analyze pupil diameter data"""
        if len(pupil_diameter) == 0:
            return {'pupil_error': 'No pupil data'}
        
        # Remove outliers
        valid_pupil = pupil_diameter[pupil_diameter > 0]
        if len(valid_pupil) == 0:
            return {'pupil_error': 'No valid pupil data'}
        
        # Remove extreme outliers (beyond 3 standard deviations)
        mean_pupil = np.mean(valid_pupil)
        std_pupil = np.std(valid_pupil)
        valid_pupil = valid_pupil[np.abs(valid_pupil - mean_pupil) <= 3 * std_pupil]
        
        return {
            'pupil_diameter_mean': np.mean(valid_pupil),
            'pupil_diameter_std': np.std(valid_pupil),
            'pupil_diameter_min': np.min(valid_pupil),
            'pupil_diameter_max': np.max(valid_pupil),
            'pupil_variability': np.std(valid_pupil) / np.mean(valid_pupil) if np.mean(valid_pupil) > 0 else 0,
            'pupil_data_quality': len(valid_pupil) / len(pupil_diameter)
        }
    
    def _analyze_fixations(self, gaze_x: np.ndarray, gaze_y: np.ndarray) -> Dict:
        """Analyze fixation patterns"""
        if len(gaze_x) == 0 or len(gaze_y) == 0:
            return {'fixation_error': 'No gaze data'}
        
        # Simple fixation detection based on velocity threshold
        velocity_threshold = 30  # degrees/second (typical threshold)
        
        # Calculate gaze velocity
        dt = 1.0 / self.sampling_rate
        vel_x = np.gradient(gaze_x) / dt
        vel_y = np.gradient(gaze_y) / dt
        velocity = np.sqrt(vel_x**2 + vel_y**2)
        
        # Identify fixations (velocity below threshold)
        fixation_mask = velocity < velocity_threshold
        
        # Find fixation periods
        fixation_starts = np.where(np.diff(fixation_mask.astype(int)) == 1)[0] + 1
        fixation_ends = np.where(np.diff(fixation_mask.astype(int)) == -1)[0] + 1
        
        # Handle edge cases
        if fixation_mask[0]:
            fixation_starts = np.concatenate([[0], fixation_starts])
        if fixation_mask[-1]:
            fixation_ends = np.concatenate([fixation_ends, [len(fixation_mask)]])
        
        # Calculate fixation metrics
        if len(fixation_starts) == 0:
            return {
                'fixation_count': 0,
                'fixation_duration_mean': 0,
                'gaze_dispersion': np.std(gaze_x) + np.std(gaze_y)
            }
        
        fixation_durations = (fixation_ends - fixation_starts) / self.sampling_rate
        
        return {
            'fixation_count': len(fixation_starts),
            'fixation_duration_mean': np.mean(fixation_durations),
            'fixation_duration_std': np.std(fixation_durations),
            'fixation_rate': len(fixation_starts) / (len(gaze_x) / self.sampling_rate),
            'gaze_dispersion': np.std(gaze_x) + np.std(gaze_y)
        }
    
    def _analyze_saccades(self, gaze_x: np.ndarray, gaze_y: np.ndarray) -> Dict:
        """Analyze saccade metrics"""
        if len(gaze_x) == 0 or len(gaze_y) == 0:
            return {'saccade_error': 'No gaze data'}
        
        # Calculate gaze velocity
        dt = 1.0 / self.sampling_rate
        vel_x = np.gradient(gaze_x) / dt
        vel_y = np.gradient(gaze_y) / dt
        velocity = np.sqrt(vel_x**2 + vel_y**2)
        
        # Saccade detection (velocity above threshold)
        saccade_threshold = 30  # degrees/second
        saccade_mask = velocity > saccade_threshold
        
        # Find saccade events
        saccade_starts = np.where(np.diff(saccade_mask.astype(int)) == 1)[0] + 1
        saccade_ends = np.where(np.diff(saccade_mask.astype(int)) == -1)[0] + 1
        
        # Handle edge cases
        if saccade_mask[0]:
            saccade_starts = np.concatenate([[0], saccade_starts])
        if saccade_mask[-1]:
            saccade_ends = np.concatenate([saccade_ends, [len(saccade_mask)]])
        
        if len(saccade_starts) == 0:
            return {
                'saccade_count': 0,
                'saccade_rate': 0,
                'saccade_amplitude_mean': 0
            }
        
        # Calculate saccade metrics
        saccade_amplitudes = []
        for start, end in zip(saccade_starts, saccade_ends):
            if end > start:
                dx = gaze_x[end-1] - gaze_x[start]
                dy = gaze_y[end-1] - gaze_y[start]
                amplitude = np.sqrt(dx**2 + dy**2)
                saccade_amplitudes.append(amplitude)
        
        return {
            'saccade_count': len(saccade_starts),
            'saccade_rate': len(saccade_starts) / (len(gaze_x) / self.sampling_rate),
            'saccade_amplitude_mean': np.mean(saccade_amplitudes) if saccade_amplitudes else 0,
            'saccade_amplitude_std': np.std(saccade_amplitudes) if len(saccade_amplitudes) > 1 else 0,
            'saccade_velocity_peak': np.max(velocity)
        }

class fNIRSProcessor:
    """
    Functional Near-Infrared Spectroscopy (fNIRS) processing
    Analyzes neural activation in prefrontal cortex
    """
    
    def __init__(self, sampling_rate: int = 10):
        self.sampling_rate = sampling_rate
        
    def process_fnirs_data(self, fnirs_data: Dict, timestamp_data: np.ndarray = None) -> Dict:
        """
        Process fNIRS data
        
        Args:
            fnirs_data: Dictionary containing oxy_hb, deoxy_hb arrays for different channels
            timestamp_data: Corresponding timestamps
            
        Returns:
            Dictionary containing fNIRS metrics
        """
        try:
            results = {
                'timestamp': time.time() if timestamp_data is None else timestamp_data[-1]
            }
            
            # Process each channel
            for channel_name, channel_data in fnirs_data.items():
                if 'oxy_hb' in channel_data and 'deoxy_hb' in channel_data:
                    oxy_hb = np.array(channel_data['oxy_hb'])
                    deoxy_hb = np.array(channel_data['deoxy_hb'])
                    
                    channel_metrics = self._analyze_fnirs_channel(oxy_hb, deoxy_hb)
                    
                    # Add channel prefix to metrics
                    for metric, value in channel_metrics.items():
                        results[f"{channel_name}_{metric}"] = value
            
            # Calculate global metrics across channels
            results.update(self._calculate_global_metrics(fnirs_data))
            
            return results
            
        except Exception as e:
            print(f"Error processing fNIRS data: {e}")
            return {'error': str(e)}
    
    def _analyze_fnirs_channel(self, oxy_hb: np.ndarray, deoxy_hb: np.ndarray) -> Dict:
        """Analyze single fNIRS channel"""
        if len(oxy_hb) == 0 or len(deoxy_hb) == 0:
            return {'error': 'No data'}
        
        # Basic signal metrics
        metrics = {
            'oxy_hb_mean': np.mean(oxy_hb),
            'oxy_hb_std': np.std(oxy_hb),
            'deoxy_hb_mean': np.mean(deoxy_hb),
            'deoxy_hb_std': np.std(deoxy_hb),
            'total_hb_mean': np.mean(oxy_hb + deoxy_hb),
            'hb_diff_mean': np.mean(oxy_hb - deoxy_hb)
        }
        
        # Activation detection (simple threshold-based)
        oxy_baseline = np.mean(oxy_hb[:min(30, len(oxy_hb))])  # First 30 samples as baseline
        oxy_activation = oxy_hb - oxy_baseline
        
        activation_threshold = 2 * np.std(oxy_activation[:min(30, len(oxy_activation))])
        activation_periods = oxy_activation > activation_threshold
        
        metrics.update({
            'activation_percentage': np.sum(activation_periods) / len(activation_periods) * 100,
            'max_activation': np.max(oxy_activation),
            'activation_slope': self._calculate_slope(oxy_activation)
        })
        
        return metrics
    
    def _calculate_global_metrics(self, fnirs_data: Dict) -> Dict:
        """Calculate global metrics across all channels"""
        all_oxy = []
        all_deoxy = []
        
        for channel_data in fnirs_data.values():
            if 'oxy_hb' in channel_data and 'deoxy_hb' in channel_data:
                all_oxy.extend(channel_data['oxy_hb'])
                all_deoxy.extend(channel_data['deoxy_hb'])
        
        if len(all_oxy) == 0:
            return {}
        
        return {
            'global_oxy_mean': np.mean(all_oxy),
            'global_deoxy_mean': np.mean(all_deoxy),
            'global_activation': np.mean(all_oxy) - np.mean(all_deoxy),
            'num_active_channels': len(fnirs_data)
        }
    
    def _calculate_slope(self, signal: np.ndarray) -> float:
        """Calculate signal slope using linear regression"""
        if len(signal) < 2:
            return 0.0
        
        x = np.arange(len(signal))
        slope, _, _, _, _ = stats.linregress(x, signal)
        return slope

class SynchronizedAnalyzer:
    """
    Synchronized analysis of all sensor modalities with robot behavioral events
    """
    
    def __init__(self):
        self.hrv_processor = HRVProcessor()
        self.eda_processor = EDAProcessor()
        self.eye_processor = EyeTrackingProcessor()
        self.fnirs_processor = fNIRSProcessor()
        
    def analyze_time_window(self, window_data: Dict, behavioral_events: List[Dict]) -> Dict:
        """
        Analyze synchronized sensor data for a time window
        
        Args:
            window_data: Dictionary containing sensor data arrays
            behavioral_events: List of behavioral events in this window
            
        Returns:
            Dictionary containing analysis results
        """
        results = {
            'window_start': window_data.get('start_time'),
            'window_end': window_data.get('end_time'),
            'window_duration': window_data.get('end_time', 0) - window_data.get('start_time', 0),
            'behavioral_events': behavioral_events
        }
        
        # Process each sensor modality
        if 'ecg' in window_data:
            hrv_results = self.hrv_processor.process_ecg_signal(
                window_data['ecg'], window_data.get('timestamps')
            )
            results['hrv'] = hrv_results
        
        if 'eda' in window_data:
            eda_results = self.eda_processor.process_eda_signal(
                window_data['eda'], window_data.get('timestamps')
            )
            results['eda'] = eda_results
        
        if 'eye_tracking' in window_data:
            eye_results = self.eye_processor.process_eye_data(
                window_data['eye_tracking'], window_data.get('timestamps')
            )
            results['eye_tracking'] = eye_results
        
        if 'fnirs' in window_data:
            fnirs_results = self.fnirs_processor.process_fnirs_data(
                window_data['fnirs'], window_data.get('timestamps')
            )
            results['fnirs'] = fnirs_results
        
        # Event-based analysis
        results['event_analysis'] = self._analyze_behavioral_events(
            behavioral_events, results
        )
        
        return results
    
    def _analyze_behavioral_events(self, events: List[Dict], sensor_results: Dict) -> Dict:
        """Analyze sensor responses to behavioral events"""
        event_analysis = {
            'num_events': len(events),
            'event_types': [event.get('event_type', 'unknown') for event in events]
        }
        
        # Analyze physiological responses to different event types
        event_types = set(event_analysis['event_types'])
        
        for event_type in event_types:
            event_count = event_analysis['event_types'].count(event_type)
            event_analysis[f'{event_type}_count'] = event_count
            
            # Add stress indicators for approach/retreat events
            if event_type in ['approach', 'retreat', 'emergency_stop']:
                stress_score = 0
                
                # HRV stress indicators
                if 'hrv' in sensor_results:
                    hrv = sensor_results['hrv']
                    if hrv.get('lf_hf_ratio', 0) > 1.5:  # High LF/HF ratio indicates stress
                        stress_score += 1
                    if hrv.get('rmssd', 0) < 20:  # Low RMSSD indicates stress
                        stress_score += 1
                
                # EDA stress indicators
                if 'eda' in sensor_results:
                    eda = sensor_results['eda']
                    if eda.get('scr_frequency', 0) > 0.1:  # High SCR frequency
                        stress_score += 1
                    if eda.get('tonic_slope', 0) > 0:  # Rising tonic level
                        stress_score += 1
                
                # Eye-tracking stress indicators
                if 'eye_tracking' in sensor_results:
                    eye = sensor_results['eye_tracking']
                    if eye.get('pupil_diameter_mean', 0) > 4.0:  # Dilated pupils
                        stress_score += 1
                    if eye.get('saccade_rate', 0) > 3.0:  # High saccade rate
                        stress_score += 1
                
                event_analysis[f'{event_type}_stress_score'] = stress_score
        
        return event_analysis
    
    def generate_summary_report(self, analysis_results: List[Dict]) -> Dict:
        """Generate summary report across multiple time windows"""
        if not analysis_results:
            return {}
        
        # Aggregate metrics across windows
        summary = {
            'total_windows': len(analysis_results),
            'total_duration': sum([r.get('window_duration', 0) for r in analysis_results]),
            'total_events': sum([r.get('event_analysis', {}).get('num_events', 0) for r in analysis_results])
        }
        
        # Average physiological metrics
        modalities = ['hrv', 'eda', 'eye_tracking', 'fnirs']
        
        for modality in modalities:
            modality_data = []
            for result in analysis_results:
                if modality in result:
                    modality_data.append(result[modality])
            
            if modality_data:
                summary[f'{modality}_summary'] = self._summarize_modality_data(modality_data)
        
        # Event-based summary
        all_events = []
        for result in analysis_results:
            if 'behavioral_events' in result:
                all_events.extend(result['behavioral_events'])
        
        summary['event_summary'] = self._summarize_events(all_events)
        
        return summary
    
    def _summarize_modality_data(self, modality_data: List[Dict]) -> Dict:
        """Summarize data for a specific modality"""
        summary = {}
        
        # Get all numeric keys
        all_keys = set()
        for data in modality_data:
            all_keys.update([k for k, v in data.items() if isinstance(v, (int, float))])
        
        # Calculate statistics for each metric
        for key in all_keys:
            values = [data[key] for data in modality_data if key in data and data[key] is not None]
            if values:
                summary[f'{key}_mean'] = np.mean(values)
                summary[f'{key}_std'] = np.std(values)
                summary[f'{key}_min'] = np.min(values)
                summary[f'{key}_max'] = np.max(values)
        
        return summary
    
    def _summarize_events(self, events: List[Dict]) -> Dict:
        """Summarize behavioral events"""
        if not events:
            return {}
        
        event_types = [event.get('event_type', 'unknown') for event in events]
        unique_types = set(event_types)
        
        summary = {
            'total_events': len(events),
            'unique_event_types': len(unique_types),
            'event_distribution': {event_type: event_types.count(event_type) 
                                 for event_type in unique_types}
        }
        
        return summary

# Test function
def test_sensor_processing():
    """Test the sensor processing system"""
    print("Testing Sensor Processing System")
    print("=" * 50)
    
    # Create test data
    duration = 30  # 30 seconds
    fs_ecg = 1000  # ECG sampling rate
    fs_eda = 1000  # EDA sampling rate
    fs_eye = 1000  # Eye tracking sampling rate
    fs_fnirs = 10  # fNIRS sampling rate
    
    # Generate synthetic physiological data
    t_ecg = np.linspace(0, duration, duration * fs_ecg)
    t_eda = np.linspace(0, duration, duration * fs_eda)
    t_eye = np.linspace(0, duration, duration * fs_eye)
    t_fnirs = np.linspace(0, duration, duration * fs_fnirs)
    
    # Synthetic ECG (simple sine wave with noise)
    ecg_signal = np.sin(2 * np.pi * 1.2 * t_ecg) + 0.1 * np.random.randn(len(t_ecg))
    
    # Synthetic EDA (gradual increase with SCR events)
    eda_signal = 5 + 0.5 * t_eda + 0.2 * np.random.randn(len(t_eda))
    scr_times = [5, 10, 15, 20]  # SCR events at these times
    for scr_time in scr_times:
        scr_idx = int(scr_time * fs_eda)
        if scr_idx < len(eda_signal):
            eda_signal[scr_idx:scr_idx+int(2*fs_eda)] += 2 * np.exp(-np.linspace(0, 3, int(2*fs_eda)))
    
    # Synthetic eye tracking data
    gaze_x = 5 * np.sin(0.5 * t_eye) + 0.5 * np.random.randn(len(t_eye))
    gaze_y = 3 * np.cos(0.3 * t_eye) + 0.3 * np.random.randn(len(t_eye))
    pupil_diameter = 3.5 + 0.5 * np.sin(0.1 * t_eye) + 0.1 * np.random.randn(len(t_eye))
    
    # Synthetic fNIRS data
    fnirs_data = {
        'ch1': {
            'oxy_hb': 0.1 * np.sin(0.05 * t_fnirs) + 0.05 * np.random.randn(len(t_fnirs)),
            'deoxy_hb': -0.05 * np.sin(0.05 * t_fnirs) + 0.02 * np.random.randn(len(t_fnirs))
        },
        'ch2': {
            'oxy_hb': 0.15 * np.sin(0.03 * t_fnirs) + 0.06 * np.random.randn(len(t_fnirs)),
            'deoxy_hb': -0.08 * np.sin(0.03 * t_fnirs) + 0.03 * np.random.randn(len(t_fnirs))
        }
    }
    
    # Create behavioral events
    behavioral_events = [
        {'timestamp': 5.0, 'event_type': 'approach', 'robot_state': 'approach'},
        {'timestamp': 10.0, 'event_type': 'observe', 'robot_state': 'observe'},
        {'timestamp': 15.0, 'event_type': 'retreat', 'robot_state': 'retreat'},
        {'timestamp': 20.0, 'event_type': 'patrol', 'robot_state': 'patrol'}
    ]
    
    # Create window data
    window_data = {
        'start_time': 0.0,
        'end_time': duration,
        'timestamps': t_ecg,
        'ecg': ecg_signal,
        'eda': eda_signal,
        'eye_tracking': {
            'gaze_x': gaze_x,
            'gaze_y': gaze_y,
            'pupil_diameter': pupil_diameter
        },
        'fnirs': fnirs_data
    }
    
    # Initialize analyzer
    analyzer = SynchronizedAnalyzer()
    
    print("Processing synchronized sensor data...")
    
    # Analyze the time window
    results = analyzer.analyze_time_window(window_data, behavioral_events)
    
    # Print results
    print("\nAnalysis Results:")
    print("-" * 30)
    
    if 'hrv' in results:
        hrv = results['hrv']
        print(f"HRV Metrics:")
        print(f"  Heart Rate: {hrv.get('heart_rate_mean', 'N/A'):.1f} bpm")
        print(f"  RMSSD: {hrv.get('rmssd', 'N/A'):.2f} ms")
        print(f"  LF/HF Ratio: {hrv.get('lf_hf_ratio', 'N/A'):.2f}")
    
    if 'eda' in results:
        eda = results['eda']
        print(f"\nEDA Metrics:")
        print(f"  Mean EDA: {eda.get('eda_mean', 'N/A'):.2f}")
        print(f"  SCR Frequency: {eda.get('scr_frequency', 'N/A'):.2f} /min")
        print(f"  Tonic Slope: {eda.get('tonic_slope', 'N/A'):.4f}")
    
    if 'eye_tracking' in results:
        eye = results['eye_tracking']
        print(f"\nEye Tracking Metrics:")
        print(f"  Pupil Diameter: {eye.get('pupil_diameter_mean', 'N/A'):.2f} mm")
        print(f"  Fixation Count: {eye.get('fixation_count', 'N/A')}")
        print(f"  Saccade Rate: {eye.get('saccade_rate', 'N/A'):.2f} /s")
    
    if 'fnirs' in results:
        print(f"\nfNIRS Metrics:")
        print(f"  Global Activation: {results['fnirs'].get('global_activation', 'N/A'):.4f}")
        print(f"  Active Channels: {results['fnirs'].get('num_active_channels', 'N/A')}")
    
    event_analysis = results.get('event_analysis', {})
    print(f"\nEvent Analysis:")
    print(f"  Total Events: {event_analysis.get('num_events', 0)}")
    print(f"  Event Types: {event_analysis.get('event_types', [])}")
    
    # Generate summary report
    summary = analyzer.generate_summary_report([results])
    print(f"\nSummary Report:")
    print(f"  Total Duration: {summary.get('total_duration', 0):.1f} seconds")
    print(f"  Total Events: {summary.get('total_events', 0)}")
    
    print("\n" + "=" * 50)
    print("Sensor processing system test completed!")

if __name__ == "__main__":
    test_sensor_processing() 