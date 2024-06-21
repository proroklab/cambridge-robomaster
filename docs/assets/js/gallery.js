function showImage(galleryId, index) {
    const gallery = document.getElementById(galleryId);
    const images = gallery.querySelectorAll('.image-container');
    images.forEach((img, i) => {
        img.classList.toggle('active', i === index);
    });
    const counter = document.getElementById('counter-' + galleryId);
    counter.textContent = `Step ${index + 1} / ${images.length}`;
}

function prevImage(galleryId) {
    const gallery = document.getElementById(galleryId);
    let currentIndex = Array.from(gallery.querySelectorAll('.image-container')).findIndex(img => img.classList.contains('active'));
    currentIndex = (currentIndex > 0) ? currentIndex - 1 : gallery.querySelectorAll('.image-container').length - 1;
    showImage(galleryId, currentIndex);
}

function nextImage(galleryId) {
    const gallery = document.getElementById(galleryId);
    let currentIndex = Array.from(gallery.querySelectorAll('.image-container')).findIndex(img => img.classList.contains('active'));
    currentIndex = (currentIndex < gallery.querySelectorAll('.image-container').length - 1) ? currentIndex + 1 : 0;
    showImage(galleryId, currentIndex);
}

// Initialize the counters when the page loads
document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('.gallery-container').forEach(gallery => {
        const galleryId = gallery.getAttribute('data-gallery');
        const images = gallery.querySelectorAll('.image-container');
        if (images.length > 0) {
            showImage(galleryId, 0); // Initialize to the first image
        }
    });
});
